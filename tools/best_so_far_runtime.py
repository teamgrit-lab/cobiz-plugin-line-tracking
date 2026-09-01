"""Reusable runtimes for the retained and realtime Mapillary profiles.

Two named profiles are intentionally kept here:

* ``swin-l-best-so-far`` is the immutable quality baseline. Selecting it
  restores the exact model revision, 384x384 model input, 640x360 score map,
  temporal alpha 0.62, and hysteresis margin 0.07 used by the retained
  full-video results.
* ``r50-fp16-640x360`` is the realtime candidate. It uses MaskFormer R50 at
  the native 640x360 camera size, FP16 on MPS/CUDA, Swin-aligned surface label
  aggregation, and a zero-cost temporal margin selected by direct comparison.

Each profile pins its own validated aggregation and temporal defaults so MCAP,
live ROS 2, and full-video tests remain reproducible.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, replace
from typing import Any

import cv2
import numpy as np
import torch
import torch.nn.functional as functional
from evaluate_mapillary_label_aggregation import (
    ROAD_LABELS,
    SIDEWALK_LABELS,
    resolve_ids,
)
from evaluate_mapillary_temporal import aggregated_selected_mask, upscale_mask
from evaluate_sidewalk_road_temporal import remove_small_components
from segment_sidewalk_road import choose_device, render_overlay
from transformers import (
    AutoImageProcessor,
    Mask2FormerForUniversalSegmentation,
    MaskFormerForInstanceSegmentation,
)

SWIN_L_PROFILE = "swin-l-best-so-far"
R50_PROFILE = "r50-fp16-640x360"
DEFAULT_PROFILE = R50_PROFILE
DEFAULT_EVALUATION_SIZE = (360, 640)
R50_ROAD_LABELS = tuple(
    label
    for label in ROAD_LABELS
    if label not in {"Bike Lane", "Parking", "Service Lane"}
)
R50_SIDEWALK_LABELS = SIDEWALK_LABELS + ("Bike Lane", "Manhole")
R50_MAXIMUM_ROAD_ISLAND_AREA = 2560
R50_MINIMUM_SIDEWALK_RING_RATIO = 0.10


@dataclass(frozen=True)
class ProfileSpec:
    """Pinned model and preprocessing contract for one runtime profile."""

    name: str
    model_family: str
    model_id: str
    model_revision: str
    input_height: int
    input_width: int
    precision: str
    temporal_alpha: float
    temporal_hysteresis_margin: float


PROFILE_SPECS = {
    SWIN_L_PROFILE: ProfileSpec(
        name=SWIN_L_PROFILE,
        model_family="mask2former",
        model_id="facebook/mask2former-swin-large-mapillary-vistas-semantic",
        model_revision="4772b6bf101d91f2534c106dc524d906aeb3c68a",
        input_height=384,
        input_width=384,
        precision="fp32",
        temporal_alpha=0.62,
        temporal_hysteresis_margin=0.07,
    ),
    R50_PROFILE: ProfileSpec(
        name=R50_PROFILE,
        model_family="maskformer",
        model_id="facebook/maskformer-resnet50-vistas",
        model_revision="ae4b8c2590c0a090fc32d5c217d78738a2dd4b19",
        input_height=360,
        input_width=640,
        precision="fp16",
        temporal_alpha=0.62,
        temporal_hysteresis_margin=0.0,
    ),
}
PROFILE_NAMES = tuple(PROFILE_SPECS)


def resolve_profile(
    name: str,
    *,
    model_id: str | None = None,
    model_revision: str | None = None,
) -> ProfileSpec:
    """Resolve a named profile while allowing explicit checkpoint overrides."""

    try:
        profile = PROFILE_SPECS[name]
    except KeyError as error:
        raise ValueError(
            f"unsupported profile '{name}'; expected one of {', '.join(PROFILE_NAMES)}"
        ) from error
    return replace(
        profile,
        model_id=model_id or profile.model_id,
        model_revision=model_revision or profile.model_revision,
    )


@dataclass(frozen=True)
class BestSoFarConfig:
    """Runtime selection plus shared temporal settings."""

    profile: str = DEFAULT_PROFILE
    model_id: str | None = None
    model_revision: str | None = None
    evaluation_height: int = DEFAULT_EVALUATION_SIZE[0]
    evaluation_width: int = DEFAULT_EVALUATION_SIZE[1]
    temporal_alpha: float | None = None
    temporal_hysteresis_margin: float | None = None
    device: str = "auto"

    def validate(self) -> None:
        resolve_profile(
            self.profile,
            model_id=self.model_id,
            model_revision=self.model_revision,
        )
        if self.model_id is not None and not self.model_id.strip():
            raise ValueError("model_id override must not be empty")
        if self.model_revision is not None and not self.model_revision.strip():
            raise ValueError("model_revision override must not be empty")
        if self.evaluation_height <= 0 or self.evaluation_width <= 0:
            raise ValueError("evaluation dimensions must be positive")
        if self.temporal_alpha is not None and not 0.5 <= self.temporal_alpha <= 1.0:
            raise ValueError("temporal_alpha must be in [0.5, 1.0]")
        if (
            self.temporal_hysteresis_margin is not None
            and not 0.0 <= self.temporal_hysteresis_margin <= 1.0
        ):
            raise ValueError("temporal_hysteresis_margin must be in [0, 1]")


@dataclass(frozen=True)
class BestSoFarResult:
    """One segmentation result and its measured processing stages."""

    selected_mask: np.ndarray
    inference_seconds: float
    postprocess_seconds: float
    total_seconds: float
    hysteresis_hold_ratio: float
    road_area_ratio: float
    sidewalk_area_ratio: float


class BestSoFarSegmenter:
    """Stateful frame segmenter for either pinned Mapillary profile."""

    def __init__(self, config: BestSoFarConfig) -> None:
        config.validate()
        self.config = config
        self.profile = resolve_profile(
            config.profile,
            model_id=config.model_id,
            model_revision=config.model_revision,
        )
        self.temporal_alpha = (
            config.temporal_alpha
            if config.temporal_alpha is not None
            else self.profile.temporal_alpha
        )
        self.temporal_hysteresis_margin = (
            config.temporal_hysteresis_margin
            if config.temporal_hysteresis_margin is not None
            else self.profile.temporal_hysteresis_margin
        )
        self.device = choose_device(config.device)
        self.use_fp16 = self.profile.precision == "fp16" and self.device.type in {
            "cuda",
            "mps",
        }
        load_started = time.perf_counter()
        self.processor = AutoImageProcessor.from_pretrained(
            self.profile.model_id,
            revision=self.profile.model_revision,
        )
        self.processor.size = {
            "height": self.profile.input_height,
            "width": self.profile.input_width,
        }
        if self.profile.model_family == "mask2former":
            self.model = Mask2FormerForUniversalSegmentation.from_pretrained(
                self.profile.model_id,
                revision=self.profile.model_revision,
            )
        elif self.profile.model_family == "maskformer":
            self.model = MaskFormerForInstanceSegmentation.from_pretrained(
                self.profile.model_id,
                revision=self.profile.model_revision,
            )
        else:  # pragma: no cover - protected by the pinned profile table
            raise ValueError(f"unsupported model family: {self.profile.model_family}")
        self.model = self.model.to(self.device)
        if self.use_fp16:
            self.model = self.model.half()
        self.model.eval()
        if self.profile.name == R50_PROFILE:
            self.road_labels = R50_ROAD_LABELS
            self.sidewalk_labels = R50_SIDEWALK_LABELS
            self.maximum_road_island_area = R50_MAXIMUM_ROAD_ISLAND_AREA
            self.minimum_sidewalk_ring_ratio = R50_MINIMUM_SIDEWALK_RING_RATIO
        else:
            self.road_labels = ROAD_LABELS
            self.sidewalk_labels = SIDEWALK_LABELS
            self.maximum_road_island_area = 0
            self.minimum_sidewalk_ring_ratio = 0.0
        self.road_ids = resolve_ids(self.model.config.id2label, self.road_labels)
        self.sidewalk_ids = resolve_ids(self.model.config.id2label, self.sidewalk_labels)
        self.model_load_seconds = time.perf_counter() - load_started
        self._previous_scores: torch.Tensor | None = None
        self._previous_selected: np.ndarray | None = None

    @property
    def evaluation_size(self) -> tuple[int, int]:
        return (self.config.evaluation_height, self.config.evaluation_width)

    def reset(self) -> None:
        """Reset temporal history between independent bags or streams."""

        self._previous_scores = None
        self._previous_selected = None

    def _move_inputs(self, inputs: dict[str, torch.Tensor]) -> dict[str, torch.Tensor]:
        moved: dict[str, torch.Tensor] = {}
        for name, value in inputs.items():
            if self.use_fp16 and value.is_floating_point():
                moved[name] = value.to(self.device, dtype=torch.float16)
            else:
                moved[name] = value.to(self.device)
        return moved

    def _semantic_scores(self, frame_bgr: np.ndarray) -> torch.Tensor:
        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        inputs = self.processor(images=frame_rgb, return_tensors="pt")
        inputs = self._move_inputs(inputs)
        with torch.inference_mode():
            outputs = self.model(**inputs)
            if self.profile.model_family == "mask2former":
                processed = self.processor.post_process_semantic_segmentation(
                    outputs,
                    target_sizes=[self.evaluation_size],
                    return_segmentation_scores=True,
                )[0]
                # Preserve the exact retained Swin-L CPU smoothing path.
                return processed["segmentation_scores"].detach().float().cpu()

            class_probabilities = outputs.class_queries_logits.softmax(dim=-1)[..., :-1]
            mask_probabilities = outputs.masks_queries_logits.sigmoid()
            scores = torch.einsum(
                "bqc,bqhw->bchw",
                class_probabilities,
                mask_probabilities,
            )
            scores = functional.interpolate(
                scores,
                size=self.evaluation_size,
                mode="bilinear",
                align_corners=False,
            )
        # Keep R50 scores on MPS/CUDA. Only maps needed by CPU morphology are
        # transferred in segment(), avoiding a 65-channel copy per frame.
        return scores[0].detach()

    def _synchronize(self) -> None:
        if self.device.type == "mps":
            torch.mps.synchronize()
        elif self.device.type == "cuda":
            torch.cuda.synchronize(self.device)

    @staticmethod
    def _as_numpy(tensor: torch.Tensor) -> np.ndarray:
        return tensor.detach().float().cpu().numpy()

    def _retain_road_components(
        self,
        road: np.ndarray,
        sidewalk: np.ndarray,
        minimum_area: int,
    ) -> np.ndarray:
        count, labels, stats, _ = cv2.connectedComponentsWithStats(
            road.astype(np.uint8), connectivity=8
        )
        retained = stats[:, cv2.CC_STAT_AREA] >= minimum_area
        retained[0] = False
        if self.maximum_road_island_area <= 0:
            return retained[labels]

        kernel = np.ones((5, 5), dtype=np.uint8)
        for component in range(1, count):
            area = int(stats[component, cv2.CC_STAT_AREA])
            if not retained[component] or area > self.maximum_road_island_area:
                continue
            component_mask = labels == component
            ring = cv2.dilate(component_mask.astype(np.uint8), kernel).astype(bool)
            ring &= ~component_mask
            ring_size = int(np.count_nonzero(ring))
            sidewalk_ratio = (
                float(np.count_nonzero(ring & sidewalk) / ring_size)
                if ring_size
                else 0.0
            )
            if sidewalk_ratio >= self.minimum_sidewalk_ring_ratio:
                retained[component] = False
        return retained[labels]

    def segment(self, frame_bgr: np.ndarray) -> BestSoFarResult:
        if frame_bgr.ndim != 3 or frame_bgr.shape[2] != 3:
            raise ValueError("frame_bgr must be an HxWx3 BGR image")

        total_started = time.perf_counter()
        scores = self._semantic_scores(frame_bgr)
        self._synchronize()
        inference_finished = time.perf_counter()

        if self._previous_scores is None:
            smooth_scores = scores
        else:
            smooth_scores = (
                self.temporal_alpha * scores
                + (1.0 - self.temporal_alpha) * self._previous_scores
            )
        self._previous_scores = smooth_scores

        smooth_map = smooth_scores.argmax(dim=0).detach().cpu().numpy()
        minimum_area = max(48, int(smooth_map.size * 0.00035))
        selected = aggregated_selected_mask(
            smooth_map,
            road_ids=self.road_ids,
            sidewalk_ids=self.sidewalk_ids,
        )

        hold_ratio = 0.0
        if (
            self._previous_selected is not None
            and self.temporal_hysteresis_margin > 0.0
        ):
            top_scores = torch.topk(smooth_scores, k=2, dim=0).values
            score_margin = self._as_numpy(top_scores[0] - top_scores[1])
            hold_mask = (selected != self._previous_selected) & (
                score_margin < self.temporal_hysteresis_margin
            )
            selected = selected.copy()
            selected[hold_mask] = self._previous_selected[hold_mask]
            hold_ratio = float(np.mean(hold_mask))
        retained_sidewalk = remove_small_components(selected == 2, minimum_area)
        retained_road = self._retain_road_components(
            selected == 1,
            retained_sidewalk,
            minimum_area,
        )
        selected.fill(0)
        selected[retained_road] = 1
        selected[retained_sidewalk] = 2
        self._previous_selected = selected

        finished = time.perf_counter()
        return BestSoFarResult(
            selected_mask=selected,
            inference_seconds=inference_finished - total_started,
            postprocess_seconds=finished - inference_finished,
            total_seconds=finished - total_started,
            hysteresis_hold_ratio=hold_ratio,
            road_area_ratio=float(np.mean(selected == 1)),
            sidewalk_area_ratio=float(np.mean(selected == 2)),
        )

    def render_overlay(
        self,
        frame_bgr: np.ndarray,
        selected_mask: np.ndarray,
        *,
        frame_index: int,
        fps: float,
    ) -> np.ndarray:
        """Render a full-resolution road/sidewalk preview."""

        return render_overlay(
            frame_bgr,
            upscale_mask(selected_mask, frame_bgr),
            frame_index=frame_index,
            fps=max(fps, 0.001),
        )

    def metadata(self) -> dict[str, Any]:
        return {
            "profile": self.profile.name,
            "model": {
                "family": self.profile.model_family,
                "id": self.profile.model_id,
                "revision": self.profile.model_revision,
            },
            "device": str(self.device),
            "precision": "fp16" if self.use_fp16 else "fp32",
            "model_load_seconds": self.model_load_seconds,
            "settings": {
                "surface_aggregate": True,
                "road_labels": list(self.road_labels),
                "sidewalk_labels": list(self.sidewalk_labels),
                "model_input_size": [
                    self.profile.input_height,
                    self.profile.input_width,
                ],
                "evaluation_size": [
                    self.config.evaluation_height,
                    self.config.evaluation_width,
                ],
                "temporal_alpha": self.temporal_alpha,
                "temporal_hysteresis_margin": self.temporal_hysteresis_margin,
                "maximum_road_island_area": self.maximum_road_island_area,
                "minimum_sidewalk_ring_ratio": self.minimum_sidewalk_ring_ratio,
            },
        }

"""Reusable runtimes for the retained and realtime Mapillary profiles.

Two named profiles are intentionally kept here:

* ``swin-l-best-so-far`` is the immutable quality baseline. Selecting it
  restores the exact model revision, 384x384 model input, 640x360 score map,
  temporal alpha 0.62, and hysteresis margin 0.07 used by the retained
  full-video results.
* ``r50-fp16-640x360`` is the realtime candidate. It uses MaskFormer R50 at
  the native 640x360 camera size and FP16 on MPS/CUDA.

Both profiles share the same Road/Sidewalk label aggregation and temporal
post-processing so MCAP, live ROS 2, and full-video tests remain comparable.
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


PROFILE_SPECS = {
    SWIN_L_PROFILE: ProfileSpec(
        name=SWIN_L_PROFILE,
        model_family="mask2former",
        model_id="facebook/mask2former-swin-large-mapillary-vistas-semantic",
        model_revision="4772b6bf101d91f2534c106dc524d906aeb3c68a",
        input_height=384,
        input_width=384,
        precision="fp32",
    ),
    R50_PROFILE: ProfileSpec(
        name=R50_PROFILE,
        model_family="maskformer",
        model_id="facebook/maskformer-resnet50-vistas",
        model_revision="ae4b8c2590c0a090fc32d5c217d78738a2dd4b19",
        input_height=360,
        input_width=640,
        precision="fp16",
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
    temporal_alpha: float = 0.62
    temporal_hysteresis_margin: float = 0.07
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
        if not 0.5 <= self.temporal_alpha <= 1.0:
            raise ValueError("temporal_alpha must be in [0.5, 1.0]")
        if not 0.0 <= self.temporal_hysteresis_margin <= 1.0:
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
        self.road_ids = resolve_ids(self.model.config.id2label, ROAD_LABELS)
        self.sidewalk_ids = resolve_ids(self.model.config.id2label, SIDEWALK_LABELS)
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
                self.config.temporal_alpha * scores
                + (1.0 - self.config.temporal_alpha) * self._previous_scores
            )
        self._previous_scores = smooth_scores

        smooth_map = smooth_scores.argmax(dim=0).detach().cpu().numpy()
        minimum_area = max(48, int(smooth_map.size * 0.00035))
        road_confidence = self._as_numpy(smooth_scores[self.road_ids].max(dim=0).values)
        sidewalk_confidence = self._as_numpy(
            smooth_scores[self.sidewalk_ids].max(dim=0).values
        )
        selected = aggregated_selected_mask(
            smooth_map,
            road_ids=self.road_ids,
            sidewalk_ids=self.sidewalk_ids,
            road_confidence=road_confidence,
            sidewalk_confidence=sidewalk_confidence,
            minimum_area=minimum_area,
        )

        hold_ratio = 0.0
        if self._previous_selected is not None:
            top_scores = torch.topk(smooth_scores, k=2, dim=0).values
            score_margin = self._as_numpy(top_scores[0] - top_scores[1])
            hold_mask = (selected != self._previous_selected) & (
                score_margin < self.config.temporal_hysteresis_margin
            )
            selected = selected.copy()
            selected[hold_mask] = self._previous_selected[hold_mask]
            retained_road = remove_small_components(selected == 1, minimum_area)
            retained_sidewalk = remove_small_components(selected == 2, minimum_area)
            selected.fill(0)
            selected[retained_road] = 1
            selected[retained_sidewalk] = 2
            hold_ratio = float(np.mean(hold_mask))
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
                "road_labels": list(ROAD_LABELS),
                "sidewalk_labels": list(SIDEWALK_LABELS),
                "model_input_size": [
                    self.profile.input_height,
                    self.profile.input_width,
                ],
                "evaluation_size": [
                    self.config.evaluation_height,
                    self.config.evaluation_width,
                ],
                "temporal_alpha": self.config.temporal_alpha,
                "temporal_hysteresis_margin": (self.config.temporal_hysteresis_margin),
            },
        }

"""Pinned R50 surface inference with unchanged label cleanup and temporal smoothing."""

from __future__ import annotations

import time
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import torch
import torch.nn.functional as functional
from segmentation_utils import (ROAD_LABELS, SIDEWALK_LABELS, resolve_ids,
                                aggregated_selected_mask, upscale_mask, remove_small_components,
                                choose_device, render_overlay)
from r50_checkpoint import (
    FINETUNED_PROFILE,
    checkpoint_manifest_sha256,
    validate_checkpoint,
)
from transformers import (
    AutoImageProcessor,
    MaskFormerForInstanceSegmentation,
)

R50_PROFILE = "r50-fp16-640x360"
R50_PROFILES = (R50_PROFILE, FINETUNED_PROFILE)
DEFAULT_PROFILE = R50_PROFILE
DEFAULT_EVALUATION_SIZE = (360, 640)
R50_ROAD_LABELS = tuple(
    label
    for label in ROAD_LABELS
    if label not in {"Bike Lane", "Parking", "Service Lane"}
)
R50_SIDEWALK_LABELS = SIDEWALK_LABELS + ("Bike Lane", "Manhole")
R50_FINETUNED_ROAD_LABELS = R50_ROAD_LABELS + ("Manhole",)
R50_FINETUNED_SIDEWALK_LABELS = tuple(
    label for label in R50_SIDEWALK_LABELS if label != "Manhole"
)
R50_MAXIMUM_ROAD_ISLAND_AREA = 2560
R50_MINIMUM_SIDEWALK_RING_RATIO = 0.10
ROAD_ISLAND_ACTIONS = ("drop", "reassign-sidewalk")
INFERENCE_BACKENDS = ("pytorch",)


@dataclass(frozen=True)
class ProfileSpec:
    """Pinned model and preprocessing contract for one runtime profile."""

    name: str
    model_family: str
    model_id: str
    model_revision: str | None
    input_height: int
    input_width: int
    precision: str
    temporal_alpha: float
    temporal_hysteresis_margin: float


PROFILE_SPECS = {
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
    FINETUNED_PROFILE: ProfileSpec(
        name=FINETUNED_PROFILE,
        model_family="maskformer",
        model_id="/models/r50-surface-v6/best",
        model_revision=None,
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
class SegmentationConfig:
    """Runtime selection plus shared temporal settings."""

    profile: str = DEFAULT_PROFILE
    model_id: str | None = None
    model_revision: str | None = None
    checkpoint_manifest_sha256: str | None = None
    evaluation_height: int = DEFAULT_EVALUATION_SIZE[0]
    evaluation_width: int = DEFAULT_EVALUATION_SIZE[1]
    temporal_alpha: float | None = None
    temporal_hysteresis_margin: float | None = None
    road_island_action: str | None = None
    minimum_sidewalk_ring_ratio: float | None = None
    pedestrian_area_road_expansion: int = 0
    device: str = "auto"
    backend: str = "pytorch"
    cuda_graph: bool = False

    def validate(self) -> None:
        if self.cuda_graph and (self.backend != 'pytorch' or self.profile not in R50_PROFILES):
            raise ValueError('CUDA replay supports only the PyTorch R50 profiles')
        resolve_profile(
            self.profile,
            model_id=self.model_id,
            model_revision=self.model_revision,
        )
        if self.model_id is not None and not self.model_id.strip():
            raise ValueError("model_id override must not be empty")
        if self.model_revision is not None and not self.model_revision.strip():
            raise ValueError("model_revision override must not be empty")
        if self.profile == FINETUNED_PROFILE:
            if self.model_revision is not None:
                raise ValueError("local R50 checkpoints do not support model_revision overrides")
            if self.backend != "pytorch":
                raise ValueError("local R50 checkpoints require the pytorch backend")
        if self.evaluation_height <= 0 or self.evaluation_width <= 0:
            raise ValueError("evaluation dimensions must be positive")
        if self.temporal_alpha is not None and not 0.5 <= self.temporal_alpha <= 1.0:
            raise ValueError("temporal_alpha must be in [0.5, 1.0]")
        if (
            self.temporal_hysteresis_margin is not None
            and not 0.0 <= self.temporal_hysteresis_margin <= 1.0
        ):
            raise ValueError("temporal_hysteresis_margin must be in [0, 1]")
        if (
            self.road_island_action is not None
            and self.road_island_action not in ROAD_ISLAND_ACTIONS
        ):
            raise ValueError(
                f"road_island_action must be one of {', '.join(ROAD_ISLAND_ACTIONS)}"
            )
        if (
            self.minimum_sidewalk_ring_ratio is not None
            and not 0.0 <= self.minimum_sidewalk_ring_ratio <= 1.0
        ):
            raise ValueError("minimum_sidewalk_ring_ratio must be in [0, 1]")
        if self.pedestrian_area_road_expansion < 0:
            raise ValueError("pedestrian_area_road_expansion must be non-negative")
        if self.backend not in INFERENCE_BACKENDS:
            raise ValueError(f"backend must be one of {', '.join(INFERENCE_BACKENDS)}")


@dataclass(frozen=True)
class SegmentationResult:
    """One segmentation result and its measured processing stages."""

    selected_mask: np.ndarray
    inference_seconds: float
    postprocess_seconds: float
    total_seconds: float
    hysteresis_hold_ratio: float
    road_area_ratio: float
    sidewalk_area_ratio: float
    road_hazard_mask: np.ndarray | None = None
    road_building_mask: np.ndarray | None = None


def _changed_pixel_hysteresis_hold_mask(
    smooth_scores: torch.Tensor,
    selected: np.ndarray,
    previous_selected: np.ndarray,
    margin: float,
) -> np.ndarray:
    """Match full-frame top-k hysteresis while scoring only changed pixels."""

    changed_mask = selected != previous_selected
    hold_mask = np.zeros_like(changed_mask)
    if not np.any(changed_mask):
        return hold_mask
    changed_positions = torch.from_numpy(changed_mask).to(device=smooth_scores.device)
    changed_scores = smooth_scores[:, changed_positions]
    top_scores = torch.topk(changed_scores, k=2, dim=0).values
    changed_margin = (top_scores[0] - top_scores[1]).detach().float().cpu().numpy()
    hold_mask[changed_mask] = changed_margin < margin
    return hold_mask


class SegmentationModel:
    """Stateful frame segmenter for either pinned Mapillary profile."""

    def __init__(self, config: SegmentationConfig) -> None:
        config.validate()
        self.config = config
        self.profile = resolve_profile(
            config.profile,
            model_id=config.model_id,
            model_revision=config.model_revision,
        )
        self.checkpoint_manifest: dict[str, Any] | None = None
        self.checkpoint_manifest_sha256: str | None = None
        if self.profile.name == FINETUNED_PROFILE:
            self.profile = replace(
                self.profile,
                model_id=str(Path(self.profile.model_id).expanduser().resolve()),
            )
            # Pin even unpinned debug loads to the manifest observed here so
            # metadata identifies the same manifest passed through validation.
            manifest_sha256 = (
                config.checkpoint_manifest_sha256
                or checkpoint_manifest_sha256(self.profile.model_id)
            )
            self.checkpoint_manifest = validate_checkpoint(
                self.profile.model_id, manifest_sha256,
            )
            self.checkpoint_manifest_sha256 = manifest_sha256.lower()
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
        self.road_island_action = config.road_island_action or "drop"
        self.pedestrian_area_road_expansion = config.pedestrian_area_road_expansion
        self.device = choose_device(config.device)
        self.use_fp16 = self.profile.precision == "fp16" and self.device.type in {
            "cuda",
            "mps",
        }
        load_started = time.perf_counter()
        if self.profile.name == FINETUNED_PROFILE:
            self.processor = AutoImageProcessor.from_pretrained(
                self.profile.model_id, local_files_only=True,
            )
        else:
            self.processor = AutoImageProcessor.from_pretrained(
                self.profile.model_id,
                revision=self.profile.model_revision,
            )
        self.processor.size = {
            "height": self.profile.input_height,
            "width": self.profile.input_width,
        }
        self.backend = config.backend
        self.model = self._load_pytorch_model()
        id2label = self.model.config.id2label
        if self.profile.name in R50_PROFILES:
            self.road_labels = (
                R50_FINETUNED_ROAD_LABELS
                if self.profile.name == FINETUNED_PROFILE else R50_ROAD_LABELS
            )
            self.sidewalk_labels = (
                R50_FINETUNED_SIDEWALK_LABELS
                if self.profile.name == FINETUNED_PROFILE else R50_SIDEWALK_LABELS
            )
            self.maximum_road_island_area = R50_MAXIMUM_ROAD_ISLAND_AREA
            self.minimum_sidewalk_ring_ratio = (
                config.minimum_sidewalk_ring_ratio
                if config.minimum_sidewalk_ring_ratio is not None
                else R50_MINIMUM_SIDEWALK_RING_RATIO
            )
        else:
            self.road_labels = ROAD_LABELS
            self.sidewalk_labels = SIDEWALK_LABELS
            self.maximum_road_island_area = 0
            self.minimum_sidewalk_ring_ratio = 0.0
        self.road_ids = resolve_ids(id2label, self.road_labels)
        self.sidewalk_ids = resolve_ids(id2label, self.sidewalk_labels)
        self.road_hazard_ids = tuple(int(k) for k, v in id2label.items() if str(v).casefold() in ('vegetation', 'curb'))
        self.road_building_ids = tuple(int(k) for k, v in id2label.items() if str(v).casefold() == 'building')
        self.pedestrian_area_id = resolve_ids(id2label, ("Pedestrian Area",))[0]
        self.model_load_seconds = time.perf_counter() - load_started
        self._previous_scores: torch.Tensor | None = None
        self._previous_selected: np.ndarray | None = None
        self._surface_lookup: torch.Tensor | None = None
        self._cuda_replay = None
        if config.cuda_graph and self.device.type != 'cuda':
            raise ValueError('CUDA replay requires a CUDA model device')

    def _load_pytorch_model(self) -> Any:
        if self.profile.name == FINETUNED_PROFILE:
            model, loading_info = MaskFormerForInstanceSegmentation.from_pretrained(
                self.profile.model_id,
                local_files_only=True,
                use_safetensors=True,
                output_loading_info=True,
            )
            problems = {
                key: loading_info[key]
                for key in ("missing_keys", "unexpected_keys", "mismatched_keys", "error_msgs")
                if loading_info.get(key)
            }
            if problems:
                raise ValueError(f"Local R50 checkpoint did not load strictly: {problems}")
        elif self.profile.model_family == "maskformer":
            model = MaskFormerForInstanceSegmentation.from_pretrained(
                self.profile.model_id,
                revision=self.profile.model_revision,
            )
        else:  # pragma: no cover - protected by the pinned profile table
            raise ValueError(f"unsupported model family: {self.profile.model_family}")
        model = model.to(self.device)
        if self.use_fp16:
            model = model.half()
        return model.eval()

    @property
    def evaluation_size(self) -> tuple[int, int]:
        return (self.config.evaluation_height, self.config.evaluation_width)

    def reset(self) -> None:
        """Reset temporal history between independent bags or streams."""

        self._previous_scores = None
        self._previous_selected = None
        self._surface_lookup = None

    def _move_inputs(self, inputs: dict[str, torch.Tensor]) -> dict[str, torch.Tensor]:
        moved: dict[str, torch.Tensor] = {}
        for name, value in inputs.items():
            if self.use_fp16 and value.is_floating_point():
                moved[name] = value.to(self.device, dtype=torch.float16)
            else:
                moved[name] = value.to(self.device)
        return moved

    def _semantic_scores(
        self, frame: np.ndarray, *, color_order: str = "bgr"
    ) -> torch.Tensor:
        frame_rgb = (
            cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) if color_order == "bgr" else frame
        )
        inputs = self.processor(images=frame_rgb, return_tensors="pt")
        inputs = self._move_inputs(inputs)
        if getattr(self.config, 'cuda_graph', False):
            from cuda_replay import CudaReplay
            if self._cuda_replay is None:
                self._cuda_replay = CudaReplay(self._r50_scores, inputs)
            return self._cuda_replay(inputs)[0].detach()
        with torch.inference_mode():
            assert self.model is not None
            outputs = self.model(**inputs)
            scores = self._r50_output_scores(outputs)
        # Keep R50 scores on MPS/CUDA. Only maps needed by CPU morphology are
        # transferred in segment(), avoiding a 65-channel copy per frame.
        return scores[0].detach()

    def _r50_scores(self, inputs):
        return self._r50_output_scores(self.model(**inputs))

    def _r50_output_scores(self, outputs):
        classes = outputs.class_queries_logits.softmax(dim=-1)[..., :-1]
        masks = outputs.masks_queries_logits.sigmoid()
        return functional.interpolate(torch.einsum('bqc,bqhw->bchw', classes, masks),
            size=self.evaluation_size, mode='bilinear', align_corners=False)

    def _synchronize(self) -> None:
        if self.device.type == "mps":
            torch.mps.synchronize()
        elif self.device.type == "cuda":
            torch.cuda.synchronize(self.device)

    def _refine_road_components(
        self,
        road: np.ndarray,
        sidewalk: np.ndarray,
        minimum_area: int,
    ) -> tuple[np.ndarray, np.ndarray]:
        count, labels, stats, _ = cv2.connectedComponentsWithStats(
            road.astype(np.uint8), connectivity=8
        )
        retained = stats[:, cv2.CC_STAT_AREA] >= minimum_area
        retained[0] = False
        if self.maximum_road_island_area <= 0:
            return retained[labels], sidewalk

        kernel = np.ones((5, 5), dtype=np.uint8)
        reassigned_sidewalk = np.zeros_like(sidewalk)
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
                if self.road_island_action == "reassign-sidewalk":
                    reassigned_sidewalk[component_mask] = True
        return retained[labels], sidewalk | reassigned_sidewalk

    def _expand_road_into_pedestrian_area(
        self,
        selected: np.ndarray,
        class_map: np.ndarray,
    ) -> np.ndarray:
        radius = self.pedestrian_area_road_expansion
        if radius <= 0:
            return selected
        road_neighborhood = cv2.dilate(
            (selected == 1).astype(np.uint8),
            np.ones((radius * 2 + 1, radius * 2 + 1), dtype=np.uint8),
        ).astype(bool)
        expanded = selected.copy()
        expanded[(class_map == self.pedestrian_area_id) & road_neighborhood] = 1
        return expanded

    def _cpu_selected_mask(self, smooth_scores: torch.Tensor) -> tuple[np.ndarray, float]:
        """Retain the CPU aggregation and changed-pixel hysteresis path."""

        smooth_map = smooth_scores.argmax(dim=0).detach().cpu().numpy()
        selected = aggregated_selected_mask(
            smooth_map, road_ids=self.road_ids, sidewalk_ids=self.sidewalk_ids
        )
        selected = self._expand_road_into_pedestrian_area(selected, smooth_map)
        hold_ratio = 0.0
        if self._previous_selected is not None and self.temporal_hysteresis_margin > 0.0:
            hold_mask = _changed_pixel_hysteresis_hold_mask(
                smooth_scores,
                selected,
                self._previous_selected,
                self.temporal_hysteresis_margin,
            )
            selected[hold_mask] = self._previous_selected[hold_mask]
            hold_ratio = float(np.mean(hold_mask))
        return selected, hold_ratio

    def _accelerator_selected_mask(
        self, smooth_scores: torch.Tensor
    ) -> tuple[np.ndarray, float]:
        """Aggregate and apply hysteresis before one compact download to CPU."""

        use_hysteresis = (
            self._previous_selected is not None and self.temporal_hysteresis_margin > 0.0
        )
        if use_hysteresis:
            # Upload the final CPU morphology result from the previous frame
            # before running this frame's class reduction and hysteresis.
            previous = torch.from_numpy(self._previous_selected).to(
                smooth_scores.device, non_blocking=True
            )
            # max, like argmax, chooses the first class on ties. topk indices
            # do not guarantee that ordering and can change the surface label.
            best_scores, class_map = smooth_scores.max(dim=0)
        else:
            class_map = smooth_scores.argmax(dim=0)
        if (
            self._surface_lookup is None
            or self._surface_lookup.device != smooth_scores.device
            or self._surface_lookup.numel() != smooth_scores.shape[0]
        ):
            lookup = torch.zeros(smooth_scores.shape[0], dtype=torch.uint8)
            lookup[self.road_ids] = 1
            lookup[self.sidewalk_ids] = 2
            self._surface_lookup = lookup.to(smooth_scores.device)
        selected = self._surface_lookup[class_map]

        radius = self.pedestrian_area_road_expansion
        if radius > 0:
            road_neighborhood = functional.max_pool2d(
                (selected == 1).float()[None, None],
                kernel_size=radius * 2 + 1,
                stride=1,
                padding=radius,
            )[0, 0].bool()
            selected = selected.masked_fill(
                (class_map == self.pedestrian_area_id) & road_neighborhood, 1
            )

        if not use_hysteresis:
            return selected.cpu().numpy(), 0.0

        # A second reduction is much cheaper on Jetson than strided full-frame
        # topk. scatter is out-of-place: never mutate the scores used by the EMA.
        second_scores = smooth_scores.scatter(
            0, class_map.unsqueeze(0), float("-inf")
        ).amax(dim=0)
        # Subtract in the original dtype, then compare in FP32, matching the
        # existing NumPy threshold even at FP16 rounding boundaries.
        score_margin = (best_scores - second_scores).float()
        hold = (selected != previous) & (score_margin < self.temporal_hysteresis_margin)
        selected = torch.where(hold, previous, selected)
        # Download two uint8 planes together (labels + hold statistics), rather
        # than an int64 class map followed by another confidence download.
        downloaded = torch.stack((selected, hold.to(torch.uint8))).cpu().numpy()
        return downloaded[0], float(np.mean(downloaded[1]))

    @torch.inference_mode()
    def segment(
        self, frame: np.ndarray, *, color_order: str = "bgr", detect_road_hazards: bool = False,
        path_mask_class: int | None = None,
    ) -> SegmentationResult:
        """Segment BGR/native RGB with surface labels independent of path mode.

        Crosswalks remain in the road group (1), including for Mode 2 requests.
        """

        # This guard must run in the calling worker thread. eval() alone does
        # not disable autograd in the hybrid model's PyTorch decoders, and the
        # temporal EMA would otherwise retain every previous frame's graph.
        if color_order not in ("bgr", "rgb"):
            raise ValueError("color_order must be 'bgr' or 'rgb'")
        if frame.ndim != 3 or frame.shape[2] != 3:
            raise ValueError("frame must be an HxWx3 image")

        if path_mask_class not in (None, 0, 1, 2):
            raise ValueError('path_mask_class must be 0, 1, 2 or None')

        total_started = time.perf_counter()
        scores = self._semantic_scores(frame, color_order=color_order)
        self._synchronize()
        inference_finished = time.perf_counter()
        hazard = building = None
        if detect_road_hazards:
            # Current logits, before temporal EMA: new grass/curb evidence wins.
            labels = scores.argmax(dim=0)
            hazard_tensor = torch.zeros_like(labels, dtype=torch.bool)
            for label in self.road_hazard_ids:
                hazard_tensor |= labels == label
            hazard = hazard_tensor.detach().cpu().numpy()
            building_tensor = torch.zeros_like(labels, dtype=torch.bool)
            for label in self.road_building_ids:
                building_tensor |= labels == label
            building = building_tensor.detach().cpu().numpy()

        if self._previous_scores is None:
            # CUDA replay reuses its output allocation. Keep temporal
            # state in a separate buffer so the next enqueue cannot overwrite it.
            smooth_scores = scores.clone() if getattr(self.config, 'cuda_graph', False) else scores
        else:
            smooth_scores = (
                self.temporal_alpha * scores
                + (1.0 - self.temporal_alpha) * self._previous_scores
            )
        self._previous_scores = smooth_scores.detach()

        if smooth_scores.device.type == "cpu":
            selected, hold_ratio = self._cpu_selected_mask(smooth_scores)
        else:
            selected, hold_ratio = self._accelerator_selected_mask(smooth_scores)
        minimum_area = max(48, int(selected.size * 0.00035))
        retained_sidewalk = remove_small_components(selected == 2, minimum_area)
        retained_road, retained_sidewalk = self._refine_road_components(
            selected == 1,
            retained_sidewalk,
            minimum_area,
        )
        selected.fill(0)
        selected[retained_road] = 1
        selected[retained_sidewalk] = 2
        self._previous_selected = selected

        finished = time.perf_counter()
        return SegmentationResult(
            selected_mask=selected,
            road_hazard_mask=hazard,
            road_building_mask=building,
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
        metadata = {
            "profile": self.profile.name,
            "model": {
                "family": self.profile.model_family,
                "id": self.profile.model_id,
                "revision": self.profile.model_revision,
            },
            "device": str(self.device),
            "precision": "fp16" if self.use_fp16 else "fp32",
            "backend": self.backend,
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
                "road_island_action": self.road_island_action,
                "pedestrian_area_road_expansion": (self.pedestrian_area_road_expansion),
            },
        }
        if self.checkpoint_manifest is not None:
            metadata["checkpoint_manifest_sha256"] = self.checkpoint_manifest_sha256
            # Live metrics need identity, not the potentially large validation
            # confusion matrix and training history stored in the artifact.
            metadata["checkpoint"] = {
                key: value for key, value in self.checkpoint_manifest.items()
                if key not in {"training", "evaluation", "note"}
            }
        return metadata

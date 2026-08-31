"""Reusable runtime for the retained Mapillary segmentation profile.

The profile implemented here is intentionally the same one used by
``segment_mapillary_full_video.py``:

* Mask2Former Swin-L trained on Mapillary Vistas
* aggregated road and sidewalk surface labels
* exponential score smoothing (alpha=0.62)
* low-margin temporal hysteresis (margin=0.07)
* small-component removal at the 640x360 evaluation resolution

Keeping the stateful post-processing in one class makes it possible to run the
same segmentation one frame at a time from MCAP or a live ROS 2 subscription.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Any

import cv2
import numpy as np
import torch
from evaluate_mapillary_label_aggregation import (
    ROAD_LABELS,
    SIDEWALK_LABELS,
    resolve_ids,
)
from evaluate_mapillary_temporal import (
    EVALUATION_SIZE,
    MODEL_ID,
    MODEL_REVISION,
    aggregated_selected_mask,
    upscale_mask,
)
from evaluate_sidewalk_road_temporal import remove_small_components
from segment_sidewalk_road import choose_device, render_overlay
from transformers import AutoImageProcessor, Mask2FormerForUniversalSegmentation


@dataclass(frozen=True)
class BestSoFarConfig:
    """Pinned model and temporal settings for the retained profile."""

    model_id: str = MODEL_ID
    model_revision: str = MODEL_REVISION
    evaluation_height: int = EVALUATION_SIZE[0]
    evaluation_width: int = EVALUATION_SIZE[1]
    temporal_alpha: float = 0.62
    temporal_hysteresis_margin: float = 0.07
    device: str = "auto"

    def validate(self) -> None:
        if not self.model_id.strip() or not self.model_revision.strip():
            raise ValueError("model id and revision must not be empty")
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
    """Stateful single-frame implementation of the retained profile."""

    def __init__(self, config: BestSoFarConfig) -> None:
        config.validate()
        self.config = config
        self.device = choose_device(config.device)
        load_started = time.perf_counter()
        self.processor = AutoImageProcessor.from_pretrained(
            config.model_id,
            revision=config.model_revision,
        )
        self.model = Mask2FormerForUniversalSegmentation.from_pretrained(
            config.model_id,
            revision=config.model_revision,
        ).to(self.device)
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

    def _semantic_scores(self, frame_bgr: np.ndarray) -> torch.Tensor:
        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        inputs = self.processor(images=frame_rgb, return_tensors="pt")
        inputs = {name: value.to(self.device) for name, value in inputs.items()}
        with torch.inference_mode():
            outputs = self.model(**inputs)
            processed = self.processor.post_process_semantic_segmentation(
                outputs,
                target_sizes=[self.evaluation_size],
                return_segmentation_scores=True,
            )[0]
        # Moving the scores to CPU also synchronizes CUDA/MPS before the timer
        # is sampled, so reported inference latency is not artificially low.
        return processed["segmentation_scores"].detach().float().cpu()

    def segment(self, frame_bgr: np.ndarray) -> BestSoFarResult:
        if frame_bgr.ndim != 3 or frame_bgr.shape[2] != 3:
            raise ValueError("frame_bgr must be an HxWx3 BGR image")

        total_started = time.perf_counter()
        scores = self._semantic_scores(frame_bgr)
        inference_finished = time.perf_counter()

        if self._previous_scores is None:
            smooth_scores = scores
        else:
            smooth_scores = (
                self.config.temporal_alpha * scores
                + (1.0 - self.config.temporal_alpha) * self._previous_scores
            )
        self._previous_scores = smooth_scores

        smooth_map = smooth_scores.argmax(dim=0).numpy()
        minimum_area = max(48, int(smooth_map.size * 0.00035))
        road_confidence = smooth_scores[self.road_ids].max(dim=0).values.numpy()
        sidewalk_confidence = (
            smooth_scores[self.sidewalk_ids].max(dim=0).values.numpy()
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
            top_scores = torch.topk(smooth_scores, k=2, dim=0).values.numpy()
            score_margin = top_scores[0] - top_scores[1]
            hold_mask = (
                (selected != self._previous_selected)
                & (score_margin < self.config.temporal_hysteresis_margin)
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
            "profile": "best-so-far",
            "model": {
                "id": self.config.model_id,
                "revision": self.config.model_revision,
            },
            "device": str(self.device),
            "model_load_seconds": self.model_load_seconds,
            "settings": {
                "surface_aggregate": True,
                "road_labels": list(ROAD_LABELS),
                "sidewalk_labels": list(SIDEWALK_LABELS),
                "evaluation_size": [
                    self.config.evaluation_height,
                    self.config.evaluation_width,
                ],
                "temporal_alpha": self.config.temporal_alpha,
                "temporal_hysteresis_margin": (
                    self.config.temporal_hysteresis_margin
                ),
            },
        }

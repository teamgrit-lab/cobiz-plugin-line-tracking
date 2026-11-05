"""Shared surface labels, mask cleanup and rendering for R50 inference."""
from __future__ import annotations

import cv2
import numpy as np
import torch


ROAD_LABELS = (
    "Road",
    "Bike Lane",
    "Crosswalk - Plain",
    "Parking",
    "Service Lane",
    "Lane Marking - Crosswalk",
    "Lane Marking - General",
)

SIDEWALK_LABELS = ("Sidewalk", "Pedestrian Area", "Curb Cut")

def resolve_ids(labels: dict[int, str] | dict[str, str], names: tuple[str, ...]) -> list[int]:
    normalized = {str(label).strip().lower(): int(raw_id) for raw_id, label in labels.items()}
    missing = [name for name in names if name.lower() not in normalized]
    if missing:
        raise RuntimeError(f"model is missing aggregation labels: {missing}")
    return [normalized[name.lower()] for name in names]

def aggregated_selected_mask(
    class_map: np.ndarray,
    *,
    road_ids: list[int],
    sidewalk_ids: list[int],
    road_confidence: np.ndarray | None = None,
    sidewalk_confidence: np.ndarray | None = None,
    minimum_confidence: float = 0.0,
    minimum_area: int | None = None,
    morph_close_kernel: int = 0,
) -> np.ndarray:
    road = np.isin(class_map, road_ids)
    sidewalk = np.isin(class_map, sidewalk_ids)
    if minimum_confidence > 0.0:
        if road_confidence is None or sidewalk_confidence is None:
            raise ValueError("confidence maps are required when thresholding")
        road &= road_confidence >= minimum_confidence
        sidewalk &= sidewalk_confidence >= minimum_confidence
    if minimum_area is not None:
        road = remove_small_components(road, minimum_area)
        sidewalk = remove_small_components(sidewalk, minimum_area)
    if morph_close_kernel:
        kernel = np.ones((morph_close_kernel, morph_close_kernel), dtype=np.uint8)
        road = cv2.morphologyEx(
            road.astype(np.uint8), cv2.MORPH_CLOSE, kernel
        ).astype(bool)
        sidewalk = cv2.morphologyEx(
            sidewalk.astype(np.uint8), cv2.MORPH_CLOSE, kernel
        ).astype(bool)
        overlap = road & sidewalk
        if np.any(overlap):
            if road_confidence is None or sidewalk_confidence is None:
                raise ValueError("confidence maps are required to resolve morphology overlap")
            road_wins = road_confidence >= sidewalk_confidence
            road[overlap] = road_wins[overlap]
            sidewalk[overlap] = ~road_wins[overlap]
    selected = np.zeros(class_map.shape, dtype=np.uint8)
    selected[road] = 1
    selected[sidewalk] = 2
    return selected

def upscale_mask(mask: np.ndarray, frame_bgr: np.ndarray) -> np.ndarray:
    if mask.shape == frame_bgr.shape[:2]:
        return mask
    return cv2.resize(
        mask,
        (frame_bgr.shape[1], frame_bgr.shape[0]),
        interpolation=cv2.INTER_NEAREST,
    )

def remove_small_components(mask: np.ndarray, minimum_area: int) -> np.ndarray:
    binary = mask.astype(np.uint8)
    count, labels, stats, _ = cv2.connectedComponentsWithStats(binary, connectivity=8)
    retained = stats[:, cv2.CC_STAT_AREA] >= minimum_area
    retained[0] = False
    return retained[labels]

ROAD_COLOR_BGR = np.asarray((40, 190, 40), dtype=np.float32)

SIDEWALK_COLOR_BGR = np.asarray((220, 60, 220), dtype=np.float32)

def render_overlay(
    frame_bgr: np.ndarray,
    selected_mask: np.ndarray,
    *,
    frame_index: int,
    fps: float,
    alpha: float = 0.48,
) -> np.ndarray:
    overlay = frame_bgr.copy()
    for class_id, color in ((1, ROAD_COLOR_BGR), (2, SIDEWALK_COLOR_BGR)):
        selected = selected_mask == class_id
        overlay[selected] = (
            overlay[selected].astype(np.float32) * (1.0 - alpha) + color * alpha
        ).astype(np.uint8)

    scale = max(0.55, min(1.1, frame_bgr.shape[1] / 1100.0))
    line_height = max(22, int(round(30 * scale)))
    panel_width = max(260, int(round(390 * scale)))
    panel_height = line_height * 3 + 16
    cv2.rectangle(
        overlay, (10, 10), (10 + panel_width, 10 + panel_height), (0, 0, 0), -1
    )
    entries = (
        ("ROAD", ROAD_COLOR_BGR.astype(np.uint8).tolist()),
        ("SIDEWALK", SIDEWALK_COLOR_BGR.astype(np.uint8).tolist()),
    )
    for row, (label, color) in enumerate(entries):
        y = 10 + line_height * (row + 1)
        cv2.line(overlay, (24, y - 6), (54, y - 6), color, max(4, int(7 * scale)))
        cv2.putText(
            overlay,
            label,
            (66, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6 * scale,
            (255, 255, 255),
            max(1, int(round(scale))),
            cv2.LINE_AA,
        )
    seconds = frame_index / fps
    cv2.putText(
        overlay,
        f"frame {frame_index} / {seconds:.2f}s",
        (24, 10 + line_height * 3),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55 * scale,
        (255, 255, 255),
        max(1, int(round(scale))),
        cv2.LINE_AA,
    )
    return overlay

def choose_device(requested: str) -> torch.device:
    if requested != "auto":
        return torch.device(requested)
    if torch.backends.mps.is_available():
        return torch.device("mps")
    if torch.cuda.is_available():
        return torch.device("cuda")
    return torch.device("cpu")

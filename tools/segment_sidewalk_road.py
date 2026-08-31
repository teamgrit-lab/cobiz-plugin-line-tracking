#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.12"
# dependencies = [
#   "numpy==2.5.2",
#   "opencv-python-headless==5.0.0.93",
#   "pillow==12.3.0",
#   "torch==2.13.0",
#   "torchvision==0.28.0",
#   "transformers==5.16.1",
# ]
# ///
"""Evaluate Road/Sidewalk semantic segmentation on a directory of videos.

The baseline uses NVIDIA's SegFormer checkpoint trained on Cityscapes.  Unlike
the repository's YOLOP model, this checkpoint has explicit ``road`` and
``sidewalk`` classes.  Sample mode deliberately produces review artifacts and
metrics before a later run spends hours processing every frame.
"""

from __future__ import annotations

import argparse
import importlib.metadata
import json
import math
import os
import platform
import shutil
import signal
import socket
import subprocess
import sys
from collections.abc import Iterable
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from itertools import pairwise
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import torch
from torch.nn import functional
from transformers import AutoImageProcessor, SegformerForSemanticSegmentation

DEFAULT_MODEL_ID = "nvidia/segformer-b0-finetuned-cityscapes-640-1280"
DEFAULT_MODEL_REVISION = "618918f3e955c8c4364d73cdbd403a40282b98b9"
VIDEO_EXTENSIONS = {".avi", ".m4v", ".mkv", ".mov", ".mp4", ".webm"}
ROAD_COLOR_BGR = np.asarray((40, 190, 40), dtype=np.float32)
SIDEWALK_COLOR_BGR = np.asarray((220, 60, 220), dtype=np.float32)
PACKAGE_NAMES = (
    "numpy",
    "opencv-python-headless",
    "pillow",
    "torch",
    "torchvision",
    "transformers",
)


@dataclass(frozen=True)
class VideoInfo:
    path: str
    stem: str
    width: int
    height: int
    fps: float
    frame_count: int
    duration_seconds: float


@dataclass(frozen=True)
class ClassMetrics:
    pixel_count: int
    area_ratio: float
    component_count: int
    small_component_count: int
    small_component_pixel_ratio: float
    boundary_per_area: float
    mean_confidence: float | None


class RunLock:
    """A small recoverable lock that prevents overlapping heartbeat runs."""

    def __init__(self, path: Path) -> None:
        self.path = path
        self.acquired = False

    @staticmethod
    def _pid_is_alive(pid: int) -> bool:
        try:
            os.kill(pid, 0)
        except ProcessLookupError:
            return False
        except PermissionError:
            return True
        return True

    def acquire(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        for _ in range(2):
            try:
                descriptor = os.open(
                    self.path,
                    os.O_CREAT | os.O_EXCL | os.O_WRONLY,
                    0o600,
                )
            except FileExistsError:
                try:
                    payload = json.loads(self.path.read_text(encoding="utf-8"))
                    pid = int(payload["pid"])
                except (OSError, ValueError, KeyError, json.JSONDecodeError):
                    pid = -1
                if pid > 0 and self._pid_is_alive(pid):
                    raise RuntimeError(
                        f"another segmentation run is active (pid={pid})"
                    )
                self.path.unlink(missing_ok=True)
                continue

            payload = {
                "pid": os.getpid(),
                "host": socket.gethostname(),
                "started_at": utc_now(),
            }
            with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
                json.dump(payload, handle, ensure_ascii=False, indent=2)
                handle.write("\n")
            self.acquired = True
            return
        raise RuntimeError(f"could not acquire run lock: {self.path}")

    def release(self) -> None:
        if self.acquired:
            self.path.unlink(missing_ok=True)
            self.acquired = False

    def __enter__(self) -> RunLock:
        self.acquire()
        return self

    def __exit__(self, *_: object) -> None:
        self.release()


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def atomic_write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def discover_videos(input_dir: Path) -> list[Path]:
    if not input_dir.is_dir():
        raise FileNotFoundError(f"input directory does not exist: {input_dir}")
    return sorted(
        path
        for path in input_dir.rglob("*")
        if path.is_file() and path.suffix.lower() in VIDEO_EXTENSIONS
    )


def inspect_video(path: Path) -> VideoInfo:
    ffprobe = shutil.which("ffprobe")
    if ffprobe is None:
        raise RuntimeError("ffprobe is required for variable-frame-rate metadata")
    completed = subprocess.run(
        [
            ffprobe,
            "-v",
            "error",
            "-select_streams",
            "v:0",
            "-show_entries",
            "stream=width,height,avg_frame_rate,duration,nb_frames",
            "-of",
            "json",
            str(path),
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    streams = json.loads(completed.stdout).get("streams", [])
    if not streams:
        raise RuntimeError(f"ffprobe found no video stream: {path}")
    stream = streams[0]
    width = int(stream["width"])
    height = int(stream["height"])
    frame_count = int(stream["nb_frames"])
    duration = float(stream["duration"])
    numerator, denominator = stream["avg_frame_rate"].split("/", maxsplit=1)
    fps = float(numerator) / float(denominator)
    if width <= 0 or height <= 0 or fps <= 0 or frame_count <= 0 or duration <= 0:
        raise RuntimeError(
            f"invalid video metadata for {path}: "
            f"{width=} {height=} {fps=} {frame_count=} {duration=}"
        )
    return VideoInfo(
        path=str(path.resolve()),
        stem=path.stem,
        width=width,
        height=height,
        fps=fps,
        frame_count=frame_count,
        duration_seconds=duration,
    )


def scene_descriptor(frame_bgr: np.ndarray) -> np.ndarray:
    reduced = cv2.resize(frame_bgr, (160, 90), interpolation=cv2.INTER_AREA)
    hsv = cv2.cvtColor(reduced, cv2.COLOR_BGR2HSV)
    histogram = cv2.calcHist([hsv], [0, 1], None, [24, 16], [0, 180, 0, 256])
    return cv2.normalize(histogram, histogram).reshape(-1)


def select_sample_frames(
    capture: cv2.VideoCapture,
    frame_count: int,
    fps: float,
    sample_count: int,
) -> list[tuple[int, float, np.ndarray, float]]:
    """Decode once, then retain endpoints/midpoint and strong scene changes.

    Sequential decoding is intentional. Random frame seeking is unreliable for
    the variable-frame-rate HEVC camera files in the target data.
    """

    candidate_count = min(frame_count, max(sample_count * 5, 25))
    requested_candidates = sorted(
        set(np.linspace(0, frame_count - 1, candidate_count, dtype=np.int64).tolist())
    )
    requested_set = set(requested_candidates)
    frames: dict[int, np.ndarray] = {}
    timestamps: dict[int, float] = {}
    last_frame: np.ndarray | None = None
    last_index = -1
    frame_index = 0
    while True:
        ok, frame = capture.read()
        if not ok or frame is None:
            break
        last_frame = frame
        last_index = frame_index
        if frame_index in requested_set:
            frames[frame_index] = frame.copy()
            timestamp = float(capture.get(cv2.CAP_PROP_POS_MSEC)) / 1000.0
            timestamps[frame_index] = (
                timestamp if timestamp >= 0.0 else frame_index / fps
            )
        frame_index += 1
    if last_frame is None:
        raise RuntimeError("video contains no decodable frames")
    if last_index not in frames:
        frames[last_index] = last_frame
        timestamp = float(capture.get(cv2.CAP_PROP_POS_MSEC)) / 1000.0
        timestamps[last_index] = timestamp if timestamp >= 0.0 else last_index / fps

    candidates = sorted(frames)
    descriptors: dict[int, np.ndarray] = {}
    for index in candidates:
        descriptors[index] = scene_descriptor(frames[index])

    changes: dict[int, float] = {candidates[0]: 0.0}
    for previous, current in pairwise(candidates):
        changes[current] = float(
            cv2.compareHist(
                descriptors[previous],
                descriptors[current],
                cv2.HISTCMP_BHATTACHARYYA,
            )
        )

    actual_midpoint = last_index // 2
    closest_midpoint = min(candidates, key=lambda index: abs(index - actual_midpoint))
    selected = {candidates[0], closest_midpoint, candidates[-1]}
    ranked_changes = sorted(changes, key=changes.get, reverse=True)
    for index in ranked_changes:
        if len(selected) >= sample_count:
            break
        selected.add(index)
    for index in candidates:
        if len(selected) >= sample_count:
            break
        selected.add(index)
    return [
        (index, timestamps[index], frames[index], changes.get(index, 0.0))
        for index in sorted(selected)
    ]


def resolve_label_id(labels: dict[int, str] | dict[str, str], name: str) -> int:
    for raw_id, label in labels.items():
        if str(label).strip().lower() == name:
            return int(raw_id)
    raise RuntimeError(f"model has no '{name}' label: {labels}")


def class_metrics(
    class_mask: np.ndarray,
    confidence: np.ndarray,
    *,
    small_component_area: int,
) -> ClassMetrics:
    binary = class_mask.astype(np.uint8)
    pixel_count = int(np.count_nonzero(binary))
    total_pixels = int(binary.size)
    if pixel_count == 0:
        return ClassMetrics(
            pixel_count=0,
            area_ratio=0.0,
            component_count=0,
            small_component_count=0,
            small_component_pixel_ratio=0.0,
            boundary_per_area=0.0,
            mean_confidence=None,
        )

    component_count, _, stats, _ = cv2.connectedComponentsWithStats(
        binary,
        connectivity=8,
    )
    areas = stats[1:, cv2.CC_STAT_AREA].astype(np.int64, copy=False)
    small = areas < small_component_area
    small_pixels = int(areas[small].sum()) if areas.size else 0
    gradient = cv2.morphologyEx(
        binary,
        cv2.MORPH_GRADIENT,
        np.ones((3, 3), dtype=np.uint8),
    )
    return ClassMetrics(
        pixel_count=pixel_count,
        area_ratio=pixel_count / total_pixels,
        component_count=max(0, component_count - 1),
        small_component_count=int(np.count_nonzero(small)),
        small_component_pixel_ratio=small_pixels / pixel_count,
        boundary_per_area=float(np.count_nonzero(gradient)) / pixel_count,
        mean_confidence=float(confidence[class_mask].mean()),
    )


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


def fit_tile(image: np.ndarray, tile_width: int) -> np.ndarray:
    height, width = image.shape[:2]
    tile_height = max(1, int(round(height * tile_width / width)))
    return cv2.resize(image, (tile_width, tile_height), interpolation=cv2.INTER_AREA)


def make_contact_sheet(
    image_paths: Iterable[Path],
    output_path: Path,
    *,
    columns: int,
    tile_width: int,
) -> None:
    images = [cv2.imread(str(path), cv2.IMREAD_COLOR) for path in image_paths]
    images = [image for image in images if image is not None]
    if not images:
        return
    tiles = [fit_tile(image, tile_width) for image in images]
    tile_height = max(tile.shape[0] for tile in tiles)
    rows = math.ceil(len(tiles) / columns)
    canvas = np.full(
        (rows * tile_height, columns * tile_width, 3),
        24,
        dtype=np.uint8,
    )
    for position, tile in enumerate(tiles):
        row, column = divmod(position, columns)
        y = row * tile_height
        x = column * tile_width
        canvas[y : y + tile.shape[0], x : x + tile.shape[1]] = tile
    output_path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(output_path), canvas):
        raise RuntimeError(f"could not write contact sheet: {output_path}")


def package_versions() -> dict[str, str]:
    return {name: importlib.metadata.version(name) for name in PACKAGE_NAMES}


def summarize_metrics(records: list[dict[str, Any]]) -> dict[str, Any]:
    summary: dict[str, Any] = {"sample_frame_count": len(records)}
    for class_name in ("road", "sidewalk"):
        populated = [
            record[class_name]
            for record in records
            if record[class_name]["pixel_count"] > 0
        ]
        all_rows = [record[class_name] for record in records]
        summary[class_name] = {
            "detected_frame_count": len(populated),
            "detected_frame_ratio": len(populated) / max(1, len(records)),
            "median_area_ratio": float(
                np.median([row["area_ratio"] for row in all_rows])
            ),
            "mean_small_component_pixel_ratio": float(
                np.mean([row["small_component_pixel_ratio"] for row in all_rows])
            ),
            "max_component_count": int(
                max((row["component_count"] for row in all_rows), default=0)
            ),
            "mean_confidence_when_present": (
                float(np.mean([row["mean_confidence"] for row in populated]))
                if populated
                else None
            ),
        }
    return summary


def write_markdown_report(
    path: Path,
    *,
    experiment_id: str,
    videos: list[VideoInfo],
    summary: dict[str, Any],
    contact_sheet: Path,
    model_id: str,
    model_revision: str,
) -> None:
    road = summary["road"]
    sidewalk = summary["sidewalk"]
    lines = [
        f"# Road/Sidewalk baseline — {experiment_id}",
        "",
        "## Result",
        "",
        "Baseline sample inference completed. This is not a success decision: "
        "dense temporal evaluation and visual review are still pending.",
        "",
        f"- Input videos: {len(videos)}",
        f"- Sample frames: {summary['sample_frame_count']}",
        f"- Road detected: {road['detected_frame_count']} frames "
        f"({road['detected_frame_ratio']:.1%})",
        f"- Sidewalk detected: {sidewalk['detected_frame_count']} frames "
        f"({sidewalk['detected_frame_ratio']:.1%})",
        f"- Mean road confidence when present: {road['mean_confidence_when_present']}",
        (
            "- Mean sidewalk confidence when present: "
            f"{sidewalk['mean_confidence_when_present']}"
        ),
        f"- Contact sheet: `{contact_sheet}`",
        "",
        "## Model provenance",
        "",
        f"- Model: `{model_id}`",
        f"- Revision: `{model_revision}`",
        (
            "- Training dataset: Cityscapes; class IDs are read from the pinned "
            "model config."
        ),
        "- Model card: https://huggingface.co/nvidia/segformer-b0-finetuned-cityscapes-640-1280",
        "- SegFormer documentation: https://huggingface.co/docs/transformers/model_doc/segformer",
        "",
        "## Remaining validation",
        "",
        "- Inspect the contact sheet for systematic domain-shift errors.",
        "- Run consecutive-frame bursts to measure mask flicker and area jumps.",
        "- Compare candidate post-processing settings without hiding semantic errors.",
        "- Process all frames only after a candidate passes the sample gate.",
    ]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def choose_device(requested: str) -> torch.device:
    if requested != "auto":
        return torch.device(requested)
    if torch.backends.mps.is_available():
        return torch.device("mps")
    if torch.cuda.is_available():
        return torch.device("cuda")
    return torch.device("cpu")


def parse_args() -> argparse.Namespace:
    repository = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--input-dir",
        type=Path,
        default=repository / "rosbag-results" / "full",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=repository / "rosbag-results" / "sidewalk-road-results",
    )
    parser.add_argument("--experiment-id", default="baseline-segformer-b0-cityscapes")
    parser.add_argument("--model-id", default=DEFAULT_MODEL_ID)
    parser.add_argument("--model-revision", default=DEFAULT_MODEL_REVISION)
    parser.add_argument("--samples-per-video", type=int, default=7)
    parser.add_argument("--device", default="auto")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.samples_per_video < 3:
        raise ValueError("--samples-per-video must be at least 3")

    input_dir = args.input_dir.resolve()
    output_dir = args.output_dir.resolve()
    experiment_dir = output_dir / "experiments" / args.experiment_id
    state_path = output_dir / "state.json"
    lock = RunLock(output_dir / "run.lock")
    stop_requested = False

    def request_stop(_signum: int, _frame: object) -> None:
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    with lock:
        videos = [inspect_video(path) for path in discover_videos(input_dir)]
        if not videos:
            raise RuntimeError(f"no videos found under {input_dir}")

        disk = shutil.disk_usage(output_dir.parent)
        preserved_state: dict[str, Any] = {}
        if state_path.is_file():
            try:
                previous_state = json.loads(state_path.read_text(encoding="utf-8"))
            except (OSError, json.JSONDecodeError):
                previous_state = {}
            for key in ("disk_policy", "video_retention_policy"):
                if key in previous_state:
                    preserved_state[key] = previous_state[key]
        initial_state: dict[str, Any] = {
            "schema_version": 1,
            "status": "running_sample_baseline",
            "updated_at": utc_now(),
            "experiment_id": args.experiment_id,
            "input_dir": str(input_dir),
            "output_dir": str(output_dir),
            "input_video_count": len(videos),
            "completed_video_count": 0,
            "api_or_account_remaining_percent": None,
            "api_or_account_usage_note": (
                "No authoritative numeric remaining-percent telemetry is exposed "
                "to this local run; the 5% rule cannot be inferred safely."
            ),
            "local_resource_snapshot": {
                "disk_free_bytes": disk.free,
                "disk_total_bytes": disk.total,
                "disk_free_ratio": disk.free / disk.total,
            },
        }
        initial_state.update(preserved_state)
        atomic_write_json(state_path, initial_state)

        device = choose_device(args.device)
        processor = AutoImageProcessor.from_pretrained(
            args.model_id,
            revision=args.model_revision,
        )
        model = SegformerForSemanticSegmentation.from_pretrained(
            args.model_id,
            revision=args.model_revision,
        ).to(device)
        model.eval()
        road_id = resolve_label_id(model.config.id2label, "road")
        sidewalk_id = resolve_label_id(model.config.id2label, "sidewalk")

        records: list[dict[str, Any]] = []
        all_overlay_paths: list[Path] = []
        for video_number, video in enumerate(videos, start=1):
            if stop_requested:
                break
            video_path = Path(video.path)
            capture = cv2.VideoCapture(str(video_path))
            video_overlay_paths: list[Path] = []
            try:
                sample_frames = select_sample_frames(
                    capture,
                    video.frame_count,
                    video.fps,
                    args.samples_per_video,
                )
                for (
                    frame_index,
                    time_seconds,
                    frame_bgr,
                    scene_change_score,
                ) in sample_frames:
                    if stop_requested:
                        break
                    frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
                    inputs = processor(images=frame_rgb, return_tensors="pt")
                    inputs = {key: value.to(device) for key, value in inputs.items()}
                    with torch.inference_mode():
                        logits = model(**inputs).logits
                        logits = functional.interpolate(
                            logits,
                            size=frame_bgr.shape[:2],
                            mode="bilinear",
                            align_corners=False,
                        )
                        probabilities = logits.softmax(dim=1)[0].cpu().numpy()
                    class_map = probabilities.argmax(axis=0)
                    road_mask = class_map == road_id
                    sidewalk_mask = class_map == sidewalk_id
                    selected_mask = np.zeros(class_map.shape, dtype=np.uint8)
                    selected_mask[road_mask] = 1
                    selected_mask[sidewalk_mask] = 2
                    small_component_area = max(32, int(selected_mask.size * 0.0002))

                    road_metrics = class_metrics(
                        road_mask,
                        probabilities[road_id],
                        small_component_area=small_component_area,
                    )
                    sidewalk_metrics = class_metrics(
                        sidewalk_mask,
                        probabilities[sidewalk_id],
                        small_component_area=small_component_area,
                    )
                    frame_record = {
                        "video": video.path,
                        "video_stem": video.stem,
                        "frame_index": frame_index,
                        "time_seconds": time_seconds,
                        "scene_change_score": scene_change_score,
                        "road": asdict(road_metrics),
                        "sidewalk": asdict(sidewalk_metrics),
                        "classes_are_disjoint": bool(
                            not np.any(road_mask & sidewalk_mask)
                        ),
                    }
                    records.append(frame_record)

                    sample_dir = experiment_dir / "samples" / video.stem
                    sample_dir.mkdir(parents=True, exist_ok=True)
                    mask_path = sample_dir / f"frame-{frame_index:08d}-mask.png"
                    overlay_path = sample_dir / f"frame-{frame_index:08d}-overlay.jpg"
                    if not cv2.imwrite(str(mask_path), selected_mask):
                        raise RuntimeError(f"could not write mask: {mask_path}")
                    overlay = render_overlay(
                        frame_bgr,
                        selected_mask,
                        frame_index=frame_index,
                        fps=video.fps,
                    )
                    if not cv2.imwrite(
                        str(overlay_path),
                        overlay,
                        [cv2.IMWRITE_JPEG_QUALITY, 91],
                    ):
                        raise RuntimeError(f"could not write overlay: {overlay_path}")
                    video_overlay_paths.append(overlay_path)
                    all_overlay_paths.append(overlay_path)

                make_contact_sheet(
                    video_overlay_paths,
                    experiment_dir / "contact-sheets" / f"{video.stem}.jpg",
                    columns=2,
                    tile_width=480,
                )
            finally:
                capture.release()

            initial_state["completed_video_count"] = video_number
            initial_state["updated_at"] = utc_now()
            initial_state["last_completed_video"] = video.path
            initial_state["sample_frame_count"] = len(records)
            atomic_write_json(state_path, initial_state)

        summary = summarize_metrics(records)
        contact_sheet = experiment_dir / "contact-sheets" / "all-videos.jpg"
        make_contact_sheet(
            all_overlay_paths,
            contact_sheet,
            columns=4,
            tile_width=360,
        )
        report = {
            "schema_version": 1,
            "experiment_id": args.experiment_id,
            "created_at": utc_now(),
            "completed": not stop_requested,
            "model": {
                "id": args.model_id,
                "revision": args.model_revision,
                "road_label_id": road_id,
                "sidewalk_label_id": sidewalk_id,
                "all_labels": {
                    str(key): value for key, value in model.config.id2label.items()
                },
            },
            "runtime": {
                "device": str(device),
                "python": platform.python_version(),
                "platform": platform.platform(),
                "packages": package_versions(),
            },
            "videos": [asdict(video) for video in videos],
            "summary": summary,
            "frames": records,
            "acceptance": {
                "success": False,
                "reason": (
                    "Baseline sample completed; visual review and dense temporal "
                    "validation are required before full-video inference."
                ),
                "visual_review": "pending",
                "dense_temporal_validation": "pending",
                "full_video_outputs": "pending",
            },
        }
        atomic_write_json(experiment_dir / "metrics.json", report)
        write_markdown_report(
            experiment_dir / "REPORT.md",
            experiment_id=args.experiment_id,
            videos=videos,
            summary=summary,
            contact_sheet=contact_sheet,
            model_id=args.model_id,
            model_revision=args.model_revision,
        )

        initial_state.update(
            {
                "status": (
                    "sample_baseline_complete"
                    if not stop_requested
                    else "paused_at_safe_checkpoint"
                ),
                "updated_at": utc_now(),
                "completed_video_count": len(videos)
                if not stop_requested
                else initial_state["completed_video_count"],
                "sample_frame_count": len(records),
                "current_experiment_report": str(experiment_dir / "REPORT.md"),
                "current_experiment_metrics": str(experiment_dir / "metrics.json"),
                "current_contact_sheet": str(contact_sheet),
                "next_action": (
                    "Visually inspect contact sheets, then run consecutive-frame "
                    "bursts and compare conservative post-processing candidates."
                ),
                "success": False,
            }
        )
        atomic_write_json(state_path, initial_state)
    return 130 if stop_requested else 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print(f"error: {error}", file=sys.stderr)
        raise

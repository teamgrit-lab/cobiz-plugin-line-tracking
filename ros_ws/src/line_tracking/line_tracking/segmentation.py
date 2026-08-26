"""ONNX inference for a single-model road and lane-line segmenter.

The default model contract is the official YOLOP export.  YOLOP has one
backbone and two segmentation heads: ``drive_area_seg`` and
``lane_line_seg``.  Both masks are therefore produced by one forward pass.
"""

from dataclasses import dataclass
from pathlib import Path
from typing import List, Tuple

import cv2
import numpy as np


@dataclass(frozen=True)
class YolopConfig:
    """Runtime settings for an OpenCV-DNN YOLOP ONNX export."""

    model_path: str
    input_width: int = 640
    input_height: int = 640
    road_threshold: float = 0.50
    line_threshold: float = 0.50
    road_gate_kernel: int = 21
    input_name: str = "images"
    road_output_name: str = "drive_area_seg"
    line_output_name: str = "lane_line_seg"

    def validate(self) -> None:
        if not self.model_path:
            raise ValueError("model_path is required for YOLOP inference")
        if self.input_width <= 0 or self.input_height <= 0:
            raise ValueError("YOLOP input dimensions must be positive")
        if not 0.0 < self.road_threshold < 1.0:
            raise ValueError("road_threshold must be in (0, 1)")
        if not 0.0 < self.line_threshold < 1.0:
            raise ValueError("line_threshold must be in (0, 1)")
        if self.road_gate_kernel <= 0 or self.road_gate_kernel % 2 == 0:
            raise ValueError("road_gate_kernel must be a positive odd number")


@dataclass(frozen=True)
class SegmentationResult:
    """Road and line masks in the original camera resolution."""

    road_mask: np.ndarray
    line_mask: np.ndarray
    raw_line_mask: np.ndarray | None = None


class YolopSegmenter:
    """Run the YOLOP multi-task ONNX model with OpenCV DNN.

    YOLOP's exported segmentation heads contain two channels (background and
    foreground) and are already sigmoid activated.  This implementation also
    accepts logits/probabilities from compatible exports by selecting the
    foreground channel and applying the configured threshold.
    """

    def __init__(self, config: YolopConfig) -> None:
        config.validate()
        model_path = Path(config.model_path)
        if not model_path.is_file():
            raise FileNotFoundError(f"YOLOP ONNX model does not exist: {model_path}")
        self.config = config
        self._net = cv2.dnn.readNetFromONNX(str(model_path))
        self._output_names = list(self._net.getUnconnectedOutLayersNames())
        if not self._output_names:
            raise RuntimeError("YOLOP ONNX model has no unconnected outputs")

    @staticmethod
    def _letterbox(
        frame_bgr: np.ndarray, target_width: int, target_height: int
    ) -> Tuple[np.ndarray, float, int, int, int, int]:
        """Match the letterbox + ImageNet normalization used by YOLOP."""

        height, width = frame_bgr.shape[:2]
        rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        ratio = min(target_width / max(width, 1), target_height / max(height, 1))
        new_width = max(1, int(round(width * ratio)))
        new_height = max(1, int(round(height * ratio)))
        resized = cv2.resize(rgb, (new_width, new_height), interpolation=cv2.INTER_AREA)

        pad_x = target_width - new_width
        pad_y = target_height - new_height
        left = pad_x // 2
        top = pad_y // 2
        canvas = np.full((target_height, target_width, 3), 114, dtype=np.uint8)
        canvas[top : top + new_height, left : left + new_width] = resized

        tensor = canvas.astype(np.float32) / 255.0
        tensor = (tensor - np.asarray([0.485, 0.456, 0.406], dtype=np.float32)) / np.asarray(
            [0.229, 0.224, 0.225], dtype=np.float32
        )
        blob = np.transpose(tensor, (2, 0, 1))[None, ...]
        return blob, ratio, left, top, new_width, new_height

    @staticmethod
    def _as_channel_first(output: np.ndarray) -> np.ndarray:
        values = np.asarray(output)
        if values.ndim == 4:
            if values.shape[1] == 2:
                return values[0]
            if values.shape[-1] == 2:
                return np.transpose(values[0], (2, 0, 1))
        if values.ndim == 3:
            if values.shape[0] == 2:
                return values
            if values.shape[-1] == 2:
                return np.transpose(values, (2, 0, 1))
        raise ValueError(
            "YOLOP segmentation output must have two channels; "
            f"received shape {values.shape}"
        )

    @staticmethod
    def _find_output(
        outputs: List[np.ndarray], names: List[str], requested: str, fallback: int
    ) -> np.ndarray:
        if requested in names:
            return outputs[names.index(requested)]
        if len(outputs) > fallback:
            return outputs[fallback]
        raise RuntimeError(
            f"YOLOP output '{requested}' was not found; available outputs: {names}"
        )

    @staticmethod
    def _foreground_probability(scores: np.ndarray) -> np.ndarray:
        foreground = scores[1].astype(np.float32, copy=False)
        # Official YOLOP exports already apply sigmoid.  Applying it here only
        # when values are outside [0, 1] also supports raw-logit exports.
        if float(np.nanmin(foreground)) < 0.0 or float(np.nanmax(foreground)) > 1.0:
            foreground = 1.0 / (1.0 + np.exp(-np.clip(foreground, -40.0, 40.0)))
        return foreground

    def _restore_mask(
        self,
        foreground: np.ndarray,
        original_shape: Tuple[int, int],
        left: int,
        top: int,
        new_width: int,
        new_height: int,
    ) -> np.ndarray:
        original_height, original_width = original_shape
        mask = (foreground >= 0.5).astype(np.uint8) * 255
        mask = cv2.resize(mask, (self.config.input_width, self.config.input_height), cv2.INTER_NEAREST)
        content = mask[top : top + new_height, left : left + new_width]
        restored = cv2.resize(content, (original_width, original_height), cv2.INTER_NEAREST)
        return restored

    def segment(self, frame_bgr: np.ndarray) -> SegmentationResult:
        if frame_bgr.ndim != 3 or frame_bgr.shape[2] != 3:
            raise ValueError("frame_bgr must be an HxWx3 image")

        blob, _, left, top, new_width, new_height = self._letterbox(
            frame_bgr, self.config.input_width, self.config.input_height
        )
        self._net.setInput(blob, self.config.input_name)
        outputs = self._net.forward(self._output_names)
        if not isinstance(outputs, (list, tuple)):
            outputs = [outputs]

        names = list(self._output_names)
        road_output = self._find_output(
            list(outputs), names, self.config.road_output_name, fallback=1
        )
        line_output = self._find_output(
            list(outputs), names, self.config.line_output_name, fallback=2
        )

        road_scores = self._as_channel_first(road_output)
        line_scores = self._as_channel_first(line_output)
        road_foreground = self._foreground_probability(road_scores)
        line_foreground = self._foreground_probability(line_scores)

        # A YOLOP lane pixel must be supported by the drivable-area mask.  A
        # small dilation absorbs the one-to-three-pixel branch misalignment at
        # the road/line boundary without admitting sidewalk vegetation.
        road_mask = self._restore_mask(
            road_foreground >= self.config.road_threshold,
            frame_bgr.shape[:2],
            left,
            top,
            new_width,
            new_height,
        )
        raw_line_mask = self._restore_mask(
            line_foreground >= self.config.line_threshold,
            frame_bgr.shape[:2],
            left,
            top,
            new_width,
            new_height,
        )
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE,
            (self.config.road_gate_kernel, self.config.road_gate_kernel),
        )
        road_gate = cv2.dilate(road_mask, kernel)
        line_mask = cv2.bitwise_and(raw_line_mask, road_gate)
        return SegmentationResult(
            road_mask=road_mask,
            line_mask=line_mask,
            raw_line_mask=raw_line_mask,
        )

import cv2
import numpy as np

from line_tracking.vision import VisionConfig, YellowLineVision


def _yellow_line_frame() -> np.ndarray:
    frame = np.full((360, 640, 3), (55, 55, 55), dtype=np.uint8)
    points = np.asarray(
        [
            [330, 359],
            [325, 310],
            [315, 260],
            [300, 210],
            [285, 165],
        ],
        dtype=np.int32,
    )
    cv2.polylines(frame, [points], False, (0, 255, 255), 18)
    return frame


def test_detects_yellow_line_and_fits_metric_path():
    detector = YellowLineVision(VisionConfig(min_component_area_px=20))

    result = detector.process(_yellow_line_frame())

    assert result.estimate is not None
    assert result.estimate.confidence >= 0.7
    assert len(result.estimate.points_xy) >= 5
    assert np.count_nonzero(result.mask) > 0
    assert abs(result.estimate.lateral(0.30)) < 0.20
    assert result.centerline_points_px.shape[1] == 2
    assert len(result.centerline_points_px) >= 2


def test_rejects_frame_without_yellow_line():
    detector = YellowLineVision(VisionConfig())
    frame = np.full((360, 640, 3), (55, 55, 55), dtype=np.uint8)

    result = detector.process(frame)

    assert result.estimate is None
    assert np.count_nonzero(result.mask) == 0


def test_robot_lateral_axis_is_positive_to_the_left():
    detector = YellowLineVision(VisionConfig(min_component_area_px=20))
    frame = np.full((360, 640, 3), (55, 55, 55), dtype=np.uint8)
    cv2.line(frame, (250, 359), (260, 165), (0, 255, 255), 18)

    result = detector.process(frame)

    assert result.estimate is not None
    assert result.estimate.lateral(0.30) > 0.0


def test_default_line_roi_is_left_right_symmetric():
    points = np.asarray(VisionConfig().line_roi_polygon, dtype=np.float32).reshape(4, 2)

    assert np.isclose(points[0, 0], 1.0 - points[1, 0])
    assert np.isclose(points[3, 0], 1.0 - points[2, 0])
    assert np.isclose(points[0, 1], points[1, 1])
    assert np.isclose(points[2, 1], points[3, 1])


def test_rejects_compact_yellow_blob_with_line_features_enabled():
    frame = np.full((360, 640, 3), (55, 55, 55), dtype=np.uint8)
    cv2.rectangle(frame, (270, 250), (370, 350), (0, 255, 255), -1)

    result = YellowLineVision(VisionConfig()).process(frame)

    assert np.count_nonzero(result.mask) == 0
    assert result.estimate is None


def test_candidate_gate_keeps_only_model_supported_yellow_pixels():
    frame = _yellow_line_frame()
    detector = YellowLineVision(VisionConfig(min_component_area_px=20))
    color_result = detector.process(frame)
    assert np.count_nonzero(color_result.mask) > 0

    gate = color_result.mask.copy()
    mixed_result = YellowLineVision(
        VisionConfig(min_component_area_px=20)
    ).process(frame, candidate_gate_mask=gate)

    assert np.count_nonzero(mixed_result.mask) > 0
    assert np.count_nonzero(mixed_result.mask[gate == 0]) == 0


def test_candidate_gate_rejects_color_line_not_found_by_model():
    frame = _yellow_line_frame()
    empty_model_line = np.zeros(frame.shape[:2], dtype=np.uint8)

    result = YellowLineVision(VisionConfig(min_component_area_px=20)).process(
        frame,
        candidate_gate_mask=empty_model_line,
    )

    assert np.count_nonzero(result.mask) == 0
    assert result.estimate is None


def test_detects_warm_cast_yellow_centerline():
    """The supplied camera records yellow paint as peach/orange."""

    frame = np.full((360, 640, 3), (115, 95, 115), dtype=np.uint8)
    points = np.asarray(
        [[360, 359], [355, 320], [345, 280], [335, 240], [325, 200]],
        dtype=np.int32,
    )
    cv2.polylines(frame, [points], False, (165, 140, 215), 14)

    result = YellowLineVision(VisionConfig(min_component_area_px=20)).process(frame)

    assert np.count_nonzero(result.mask) > 0
    assert result.estimate is not None

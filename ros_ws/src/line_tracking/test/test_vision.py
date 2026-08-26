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

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


def _gray_road_frame() -> np.ndarray:
    frame = np.zeros((360, 640, 3), dtype=np.uint8)
    frame[:, :] = (45, 90, 45)
    road_polygon = np.asarray(
        [[64, 359], [575, 359], [415, 162], [223, 162]],
        dtype=np.int32,
    )
    cv2.fillPoly(frame, [road_polygon], (110, 110, 110))
    cv2.line(frame, (300, 350), (330, 205), (245, 245, 245), 6)
    return frame


def _colored_lane_frame() -> np.ndarray:
    frame = np.full((360, 640, 3), (55, 55, 55), dtype=np.uint8)
    yellow_points = np.asarray(
        [[250, 359], [260, 320], [275, 280], [290, 240], [305, 200]],
        dtype=np.int32,
    )
    white_points = np.asarray(
        [[410, 359], [400, 320], [390, 280], [380, 240], [370, 200]],
        dtype=np.int32,
    )
    cv2.polylines(frame, [yellow_points], False, (0, 255, 255), 12)
    cv2.polylines(frame, [white_points], False, (255, 255, 255), 12)
    return frame


def _road_lane_frame() -> np.ndarray:
    """Synthetic perspective road with a yellow left and white right lane."""

    frame = np.full((360, 640, 3), (55, 55, 55), dtype=np.uint8)
    left_points = np.asarray(
        [[110, 359], [145, 320], [185, 275], [225, 225], [265, 162]],
        dtype=np.int32,
    )
    right_points = np.asarray(
        [[530, 359], [495, 320], [455, 275], [415, 225], [375, 162]],
        dtype=np.int32,
    )
    cv2.polylines(frame, [left_points], False, (0, 255, 255), 14)
    cv2.polylines(frame, [right_points], False, (255, 255, 255), 14)
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


def test_adaptive_gray_road_mask_stays_inside_roi():
    detector = YellowLineVision(VisionConfig())

    mask = detector.detect_gray_road_mask(_gray_road_frame())

    assert mask[350, 300] > 0
    assert mask[190, 320] == 0
    assert mask[100, 320] == 0
    assert mask[350, 20] == 0


def test_detects_lines_only_inside_gray_road_mask():
    detector = YellowLineVision(VisionConfig())
    frame = _gray_road_frame()
    road_mask = detector.detect_gray_road_mask(frame)

    line_mask = detector.detect_lines_in_mask(frame, road_mask)

    assert np.count_nonzero(line_mask) > 0
    assert np.count_nonzero(line_mask[road_mask == 0]) == 0


def test_detects_yellow_and_white_lane_segments_without_path_fitting():
    detector = YellowLineVision(VisionConfig())

    result = detector.detect_lane_lines(_colored_lane_frame())

    assert np.count_nonzero(result.yellow_color_mask) > 0
    assert np.count_nonzero(result.white_color_mask) > 0
    assert np.count_nonzero(result.yellow_line_mask) > 0
    assert np.count_nonzero(result.white_line_mask) > 0
    assert np.count_nonzero(result.yellow_line_mask & result.white_line_mask) == 0


def test_lane_segments_are_gated_by_an_external_road_mask():
    detector = YellowLineVision(VisionConfig())
    frame = _colored_lane_frame()
    road_mask = np.zeros(frame.shape[:2], dtype=np.uint8)
    road_polygon = np.asarray(
        [[64, 359], [575, 359], [415, 162], [223, 162]],
        dtype=np.int32,
    )
    cv2.fillPoly(road_mask, [road_polygon], 255)

    result = detector.detect_lane_lines(frame, road_mask=road_mask)

    detected = result.yellow_line_mask | result.white_line_mask
    assert np.count_nonzero(detected) > 0
    assert np.count_nonzero(detected[road_mask == 0]) == 0


def test_advanced_lane_backend_fits_both_lanes_and_road_corridor():
    detector = YellowLineVision(VisionConfig())

    result = detector.detect_advanced_lanes(_road_lane_frame())

    assert result.detected
    assert result.confidence >= 0.70
    assert result.left_curve_px.shape[1] == 2
    assert result.right_curve_px.shape[1] == 2
    assert result.centerline_points_px.shape[1] == 2
    assert len(result.lane_polygon_px) >= 4
    assert result.center_offset_m is not None
    assert abs(result.center_offset_m) < 0.25
    assert np.count_nonzero(result.binary_mask) > 0
    assert np.count_nonzero(result.birdseye_mask) > 0


def test_advanced_lane_backend_rejects_frame_without_lane_pair():
    detector = YellowLineVision(VisionConfig())
    frame = np.full((360, 640, 3), (55, 55, 55), dtype=np.uint8)

    result = detector.detect_advanced_lanes(frame)

    assert not result.detected
    assert result.confidence == 0.0
    assert result.curvature_m is None
    assert result.center_offset_m is None

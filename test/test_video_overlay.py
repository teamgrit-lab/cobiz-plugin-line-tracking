import numpy as np

from line_tracking.segmentation import SegmentationResult
from line_tracking.vision import VisionConfig, YellowLineVision
from tools.segment_video import render_opencv_overlay, render_segmentation_overlay, select_profile


def test_auto_profile_matches_video_height():
    assert select_profile("auto", 1280, 720) == ("720p", 1280, 736)
    assert select_profile("auto", 640, 360) == ("360p", 640, 384)


def test_overlay_keeps_masks_visibly_distinct():
    frame = np.zeros((20, 30, 3), dtype=np.uint8)
    road = np.zeros((20, 30), dtype=np.uint8)
    raw_line = np.zeros((20, 30), dtype=np.uint8)
    gated_line = np.zeros((20, 30), dtype=np.uint8)
    road[2:8, 2:8] = 255
    raw_line[10:14, 2:8] = 255
    gated_line[10:14, 2:8] = 255

    output = render_segmentation_overlay(
        frame,
        SegmentationResult(
            road_mask=road,
            raw_line_mask=raw_line,
            line_mask=gated_line,
        ),
        show_legend=False,
    )

    road_pixel = output[4, 4]
    line_pixel = output[11, 4]
    assert road_pixel[1] > road_pixel[0]
    assert line_pixel[1] > line_pixel[0]
    assert line_pixel[2] > line_pixel[0]
    assert output.shape == frame.shape


def test_opencv_overlay_marks_yellow_mask_and_roi():
    frame = np.zeros((40, 60, 3), dtype=np.uint8)
    vision = YellowLineVision(VisionConfig())
    result = vision.process(frame)
    output = render_opencv_overlay(frame, result, show_legend=False)
    assert output.shape == frame.shape
    assert result.road_mask is None
    assert result.raw_line_mask is None

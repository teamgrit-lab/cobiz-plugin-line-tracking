import numpy as np

from line_tracking.segmentation import SegmentationResult
from tools.segment_video import render_segmentation_overlay, select_profile


def test_auto_profile_matches_video_height():
    assert select_profile("auto", 1280, 720) == ("720p", 1280, 720)
    assert select_profile("auto", 640, 360) == ("360p", 640, 360)


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

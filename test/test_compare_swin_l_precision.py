from pathlib import Path
import sys

import numpy as np
import pytest


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

from compare_swin_l_precision import (  # noqa: E402
    compare_masks,
    compare_paths,
    memory_ratios,
)


def test_compare_masks_reports_literal_agreement_and_class_iou():
    fp32 = np.asarray(
        (
            ((0, 1), (2, 2)),
            ((0, 1), (2, 2)),
        ),
        dtype=np.uint8,
    )
    fp16 = np.asarray(
        (
            ((0, 1), (2, 2)),
            ((0, 0), (2, 2)),
        ),
        dtype=np.uint8,
    )

    result = compare_masks(fp32, fp16)

    assert result["selected_mask_agreement"] == pytest.approx(7 / 8)
    assert result["road_iou"] == pytest.approx(0.5)
    assert result["sidewalk_iou"] == pytest.approx(1.0)
    assert result["exact_frame_ratio"] == pytest.approx(0.5)


def test_compare_paths_counts_validity_mismatch_and_lateral_error():
    fp32_valid = np.asarray((True, True, False))
    fp16_valid = np.asarray((True, False, False))
    fp32_points = np.full((3, 2, 2), np.nan, dtype=np.float32)
    fp16_points = np.full((3, 2, 2), np.nan, dtype=np.float32)
    fp32_points[0] = ((3.0, 0.0), (4.0, 0.1))
    fp16_points[0] = ((3.0, 0.05), (4.0, 0.2))
    fp32_confidence = np.asarray((0.9, 0.8, np.nan), dtype=np.float32)
    fp16_confidence = np.asarray((0.85, np.nan, np.nan), dtype=np.float32)

    result = compare_paths(
        fp32_valid,
        fp32_points,
        fp32_confidence,
        fp16_valid,
        fp16_points,
        fp16_confidence,
    )

    assert result["validity_agreement"] == pytest.approx(2 / 3)
    assert result["validity_mismatch_count"] == 1
    assert result["paired_valid_frames"] == 1
    assert result["lateral_error_m"]["mean"] == pytest.approx(0.075)
    assert result["lateral_error_m"]["p95"] == pytest.approx(0.0975)
    assert result["lateral_error_m"]["max"] == pytest.approx(0.1)
    assert result["confidence_abs_error"]["max"] == pytest.approx(0.05)


def test_memory_ratios_use_fp32_as_denominator():
    fp32 = {
        "floating_parameter_bytes": 1000,
        "activation_peak_allocated_bytes": 600,
        "overall_peak_allocated_bytes": 2000,
    }
    fp16 = {
        "floating_parameter_bytes": 500,
        "activation_peak_allocated_bytes": 360,
        "overall_peak_allocated_bytes": 1200,
    }

    result = memory_ratios(fp32, fp16)

    assert result == {
        "floating_parameter_ratio": 0.5,
        "activation_peak_allocated_ratio": 0.6,
        "overall_peak_allocated_ratio": 0.6,
    }

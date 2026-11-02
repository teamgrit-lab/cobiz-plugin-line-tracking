"""Recorded training/evidence point clouds retain their measured mount and layout."""
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from lidar_common import a2_livox_front_transforms, cloud_xyz, validate_transform
from sensor_fixtures import cloud


@pytest.mark.parametrize("endian", ["<", ">"])
def test_decoder_respects_mid360_stride_endian_and_padding(endian):
    packet = cloud([[1, 2, 3], [4, 5, 6]], endian=endian, padding=12)
    np.testing.assert_allclose(cloud_xyz(packet), [[1, 2, 3], [4, 5, 6]])
    packet.data = packet.data[:-1]
    with pytest.raises(ValueError):
        cloud_xyz(packet)


def test_recorded_cloud_drops_invalid_returns_and_keeps_near_points():
    decoded = cloud_xyz(cloud([[0, 0, 0], [np.nan, 0, 0], [0.1, 0, 0]]))
    np.testing.assert_allclose(decoded, [[0.1, 0, 0]])


def test_recorded_calibration_requires_correct_frames_and_rigid_rotation():
    matrix = np.eye(4)
    matrix[0, 0] = 2
    with pytest.raises(ValueError):
        validate_transform(matrix)
    with pytest.raises(ValueError, match="frame_mismatch"):
        a2_livox_front_transforms("wrong", "base_link", "camera_optical_frame")

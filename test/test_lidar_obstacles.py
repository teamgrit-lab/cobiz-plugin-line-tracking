import sys
from pathlib import Path
from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from lidar_common import a2_livox_front_transforms, cloud_xyz, parse_transform
from lidar_obstacles import ObstacleConfig, ObstacleDetector, camera_region, fresh
from sensor_fixtures import cloud, imu, camera_info, header


def test_measured_tilted_mount_slices_about_lidar_origin_along_gravity():
    base, _ = a2_livox_front_transforms(
        "livox_frame", "base_link", "camera_optical_frame"
    )
    heights = np.array([-0.101, -0.1, 0, 0.1, 0.101, -0.4])
    points = np.column_stack(
        (np.arange(6) * 0.2 + 2, np.zeros(6), base[2, 3] + heights)
    )
    raw = (points - base[:3, 3]) @ base[:3, :3]
    up = base[:3, :3].T @ np.array([0, 0, 1.0])
    config = ObstacleConfig(
        height_reference="lidar_gravity", min_height_m=-0.1, max_height_m=0.1,
        horizontal_range_m=6.0, fov_deg=360.0,
    )
    scan = ObstacleDetector(config).update(cloud(raw), imu(up=up), base, 0.0)
    np.testing.assert_allclose(scan.up_base, [0, 0, 1], atol=1e-6)
    np.testing.assert_allclose(scan.points[:, 0], [2.2, 2.4, 2.6], atol=0.01)
    assert scan.input_count == 6


def test_default_height_band_uses_base_link_z_after_tilted_mount_transform():
    base, _ = a2_livox_front_transforms("livox_frame", "base_link", "camera_optical_frame")
    points = np.array([
        [.8, -.4, -.201], [.8, -.4, -.2], [.9, .4, 0],
        [1., .4, .2], [1.1, .4, .201],
        [1.2, -.4, base[2, 3] + .1], [1.2, .4, -.15],
    ])
    raw = (points - base[:3, 3]) @ base[:3, :3]
    # Body-frame z must stay fixed even when instantaneous acceleration tilts.
    up = base[:3, :3].T @ np.array([.25, .15, np.sqrt(1 - .25**2 - .15**2)])
    scan = ObstacleDetector(ObstacleConfig()).update(cloud(raw), imu(up=up), base, 0.0)
    np.testing.assert_allclose(scan.points, [[.8, -.4], [.9, .4], [1., .4], [1.2, .4]], atol=1e-6)


def test_default_sector_is_horizontal_base_radius_and_robot_forward_190_degrees():
    base, _ = a2_livox_front_transforms("livox_frame", "base_link", "camera_optical_frame")
    points = np.array([
        [1.5, 0, 0], [1.501, 0, 0], [0, 1.5, 0], [0, -1.5, 0],
        [-.01, 1., 0], [1.2, 1.2, 0], [1.4, .4, 0], [1.6, .1, 0],
    ])
    # Include both ±95° boundaries, exclude returns just outside at ±95.1°.
    angles = np.deg2rad([95, -95, 95.1, -95.1])
    radii = np.array([1.2, 1.2, .8, .8])
    points = np.vstack((points, np.column_stack((
        radii * np.cos(angles), radii * np.sin(angles), np.zeros(4),
    ))))
    raw = (points - base[:3, 3]) @ base[:3, :3]
    scan = ObstacleDetector(ObstacleConfig()).update(
        cloud(raw), imu(up=base[:3, :3].T @ [0, 0, 1]), base, 0.0
    )
    np.testing.assert_allclose(scan.points, [
        [-.1, -1.2], [-.1, 1.2], [0, -1.5], [0, 1.], [0, 1.5], [1.4, .4], [1.5, 0],
    ], atol=1e-6)


def test_far_returns_observe_near_sector_without_clearing_outside_it():
    config = ObstacleConfig()
    angles = np.linspace(-np.pi, np.pi, 1440, endpoint=False)
    base, _ = a2_livox_front_transforms("livox_frame", "base_link", "camera_optical_frame")
    points = np.column_stack((100 * np.cos(angles), 100 * np.sin(angles), np.zeros(len(angles))))
    raw = (points - base[:3, 3]) @ base[:3, :3]
    scan = ObstacleDetector(config).update(
        cloud(raw), imu(up=base[:3, :3].T @ [0, 0, 1]), base, 0.0
    )
    assert not len(scan.points)
    for xy in [[1., 0], [.6, .8], [.6, -.8], [-.1, 1.4], [-.1, -1.4]]:
        assert scan.observed[tuple(config.indices([xy])[0])]
    assert not scan.observed[~config.in_detection_sector(config.centers())].any()


def test_base_height_ray_starts_at_actual_lidar_height():
    config = ObstacleConfig()
    base = np.eye(4)
    base[2, 3] = .4
    # The ray enters the z<=0.2 band only after x=0.56m.
    scan = ObstacleDetector(config).update(cloud([[1.4, 0, -.5]]), imu(), base, 0.0)
    near, far = config.indices([[.4, 0], [1., 0]])
    assert not scan.observed[tuple(near)]
    assert scan.observed[tuple(far)]


@pytest.mark.parametrize("endian", ["<", ">"])
def test_decoder_respects_mid360_stride_endian_and_padding(endian):
    packet = cloud([[1, 2, 3], [4, 5, 6]], endian=endian, padding=12)
    np.testing.assert_allclose(cloud_xyz(packet), [[1, 2, 3], [4, 5, 6]])
    packet.data = packet.data[:-1]
    with pytest.raises(ValueError):
        cloud_xyz(packet)


def test_invalid_returns_are_removed_without_blinding_nearby_obstacles():
    decoded = cloud_xyz(cloud([[0, 0, 0], [np.nan, 0, 0], [0.1, 0, 0]]))
    np.testing.assert_allclose(decoded, [[0.1, 0, 0]])
    base, _ = a2_livox_front_transforms(
        "livox_frame", "base_link", "camera_optical_frame"
    )
    point = np.array([[0.55, 0.0, base[2, 3]]])
    raw = (point - base[:3, 3]) @ base[:3, :3]
    assert np.linalg.norm(raw) < 0.2
    scan = ObstacleDetector(ObstacleConfig()).update(
        cloud(raw), imu(up=base[:3, :3].T @ [0, 0, 1]), base, 0.0
    )
    assert len(scan.points) == 1


def test_nonrigid_mount_and_invalid_profile_frames_are_rejected():
    matrix = np.eye(4)
    matrix[0, 0] = 2
    with pytest.raises(ValueError):
        parse_transform(",".join(map(str, matrix.ravel())))
    with pytest.raises(ValueError, match="frame_mismatch"):
        a2_livox_front_transforms("wrong", "base_link", "camera_optical_frame")


@pytest.mark.parametrize(
    "unit,magnitude", [("auto", 1.0), ("auto", 9.80665), ("g", 1.0), ("mps2", 9.80665)]
)
def test_livox_g_and_standard_ros_acceleration_give_the_same_gravity(unit, magnitude):
    scan = ObstacleDetector(replace(ObstacleConfig(), imu_accel_unit=unit)).update(
        cloud([[2, 0, 0]]), imu(up=(0, 0, magnitude)), np.eye(4), 0.0
    )
    np.testing.assert_allclose(scan.up_base, [0, 0, 1])


def test_rays_do_not_clear_space_beyond_an_obstacle_or_outside_slice():
    config = ObstacleConfig()
    scan = ObstacleDetector(config).update(
        cloud([[1.2, 0, 0], [4, 1, -0.8]]), imu(), np.eye(4), 0.0
    )
    a, b, c = config.indices([[1, 0], [1.4, 0], [1.4, 0.35]])
    assert scan.observed[tuple(a)]
    assert not scan.observed[tuple(b)]
    assert not scan.observed[tuple(c)]


def test_empty_invalid_and_repeated_scans_are_faults_but_filtered_floor_is_valid():
    detector = ObstacleDetector(ObstacleConfig())
    with pytest.raises(ValueError, match="points_unavailable"):
        detector.update(cloud([]), imu(), np.eye(4), 0.0)
    floor = detector.update(cloud([[2, 0, -0.4]]), imu(), np.eye(4), 0.0)
    assert not len(floor.points)
    with pytest.raises(ValueError, match="timestamp_invalid"):
        detector.update(cloud([[2, 0, 0]]), imu(), np.eye(4), 0.1)
    assert fresh(floor, 0.31, 100_310_000_000, detector.config) == "lidar_stale"


def test_latest_body_frame_returns_replace_previous_scan_without_pose():
    detector = ObstacleDetector(ObstacleConfig())
    detector.update(cloud([[1.2, 0, 0]]), imu(), np.eye(4), 0.0)
    scan = detector.update(cloud([[1.1, 0, 0]], 100.1), imu(100.1), np.eye(4), 0.1)
    np.testing.assert_allclose(scan.points, [[1.1, 0]], atol=1e-5)
    assert len(scan.points) == 1


def test_bad_imu_is_rejected_without_ground_fitting():
    d = ObstacleDetector(ObstacleConfig())
    for sample in (imu(up=(0, 0, 0)), imu(frame="other"), imu(stamp=99.0)):
        with pytest.raises(ValueError):
            d.update(cloud([[2, 0, 0]]), sample, np.eye(4), 0.0)


def test_metric_camera_projection_uses_intrinsics_and_selected_class():
    config = ObstacleConfig()
    base, cam = a2_livox_front_transforms(
        "livox_frame", "base_link", "camera_optical_frame"
    )
    scan = ObstacleDetector(config).update(
        cloud([[2, 0, 0]]), imu(up=base[:3, :3].T @ [0, 0, 1]), base, 0.0
    )
    mask = np.full((360, 640), 2, np.uint8)
    region = camera_region(mask, camera_info(), cam, scan, config, 2)
    point = config.indices([[2, 0]])[0]
    assert region[tuple(point)]
    assert not camera_region(mask, camera_info(), cam, scan, config, 1).any()
    assert camera_region(mask, camera_info(), cam, scan, config, 0)[tuple(point)]


def test_runtime_caches_fixed_extrinsics_and_rejects_older_camera_references():
    from lidar_runtime import LidarRuntime
    from local_avoidance import AvoidanceConfig

    base, camera = a2_livox_front_transforms(
        "livox_frame", "base_link", "camera_optical_frame"
    )
    calls = []

    def lookup(target, source, stamp):
        calls.append((target, source))
        return base if target == "base_link" else camera

    args = SimpleNamespace(
        lidar_to_base_transform="",
        base_to_camera_transform="",
        lidar_calibration_profile="tf",
        path_frame_id="base_link",
        lidar_frame_id="livox_frame",
    )
    runtime = LidarRuntime(args, ObstacleConfig(), AvoidanceConfig(), lookup)
    for stamp in [100.0, 100.1]:
        runtime.on_imu(imu(stamp, up=base[:3, :3].T @ [0, 0, 1]), stamp - 100)
        runtime.on_cloud(cloud([[2, 0, 0]], stamp), stamp - 100, round(stamp * 1e9))
        assert runtime.fault is None
    runtime.on_info(camera_info())
    for stamp in [100.0, 100.1]:
        runtime.set_reference(
            np.zeros((360, 640), np.uint8),
            header(stamp, "camera_optical_frame"),
            (360, 640),
            stamp - 100,
        )
        runtime._context(2, 0.1, 100_100_000_000)
    runtime.set_reference(
        np.full((360, 640), 2, np.uint8),
        header(99.0, "camera_optical_frame"),
        (360, 640),
        0.1,
    )
    _, region = runtime._context(2, 0.1, 100_100_000_000)
    assert not region[tuple(runtime.config.indices([[2, 0]])[0])]
    assert calls == [
        ("base_link", "livox_frame"),
        ("camera_optical_frame", "base_link"),
    ]


@pytest.mark.parametrize(
    "change",
    [
        {"min_height_m": 0.0},
        {"max_age_sec": float("nan")},
        {"resolution_m": 0.001},
        {"height_reference": "sensor_z"},
        {"horizontal_range_m": 0.0},
        {"horizontal_range_m": float("nan")},
        {"fov_deg": 0.0},
        {"fov_deg": 361.0},
    ],
)
def test_invalid_obstacle_configuration_fails_before_driving(change):
    with pytest.raises(ValueError):
        replace(ObstacleConfig(), **change).validate()

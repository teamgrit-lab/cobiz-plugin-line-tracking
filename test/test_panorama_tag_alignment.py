"""Offline spherical geometry, planar pose ambiguity and temporal confirmation."""
import json
from pathlib import Path
import sys

import cv2
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from panorama_tag_alignment import (
    AlignmentMonitor, PanoramaDetector, bearing_error, estimate_plane, main,
    marker_center, plane_error, write_preview_sample,
)


@pytest.mark.parametrize(('x', 'axis', 'expected'), [
    (720, 720, 0), (2160, 2160, 0), (728, 720, 1),
    (712, 720, -1), (2879, 0, -.125), (1, 2879, .25),
])
def test_bearing_uses_source_pixels_and_wraps_at_panorama_seam(x, axis, expected):
    assert bearing_error(x, axis, 2880) == pytest.approx(expected)


def test_plane_orientation_is_independent_of_which_way_the_normal_points():
    assert plane_error(0, 90, parallel=True) == 0
    assert plane_error(180, 90, parallel=True) == 0
    assert plane_error(90, 90, parallel=False) == 0
    assert abs(plane_error(90, 90, parallel=True)) == 90


def detection(x=720, tag_id=7, parallel_error=0.):
    return dict(tag_id=tag_id, panorama_x=x, panorama_y=720., pose_status='ok',
                parallel_error_deg=parallel_error)


def test_center_and_parallel_confirmation_are_separate_and_missing_tag_resets():
    monitor = AlignmentMonitor(2880, 720, 2160)
    # The tag is to the side, with a plane parallel to the forward axis.
    for time in (0., .2, .4):
        result = monitor.observe([detection(x=1440)], time)[0]
    assert result['nearest_axis'] == 'equidistant'
    assert not result['horizontal_centered']
    assert result['parallel_aligned']
    assert monitor.observe([], .6) == []
    result = monitor.observe([detection(x=1440)], .8)[0]
    assert result['parallel_confirm_frames'] == 1
    assert not result['parallel_aligned']


def test_rear_tag_centering_does_not_assert_parallel_heading():
    monitor = AlignmentMonitor(2880, 720, 2160)
    for time in (0., .2, .4):
        result = monitor.observe([detection(x=2160, parallel_error=45.)], time)[0]
    assert result['horizontal_centered']
    assert result['nearest_axis'] == 'rear'
    assert not result['parallel_aligned']


def test_repeated_frame_and_large_gap_cannot_complete_confirmation():
    monitor = AlignmentMonitor(2880, 720, 2160)
    monitor.observe([detection()], 0.)
    with pytest.raises(ValueError, match='timestamps must increase'):
        monitor.observe([detection()], 0.)
    result = monitor.observe([detection()], 2.)[0]
    assert result['center_confirm_frames'] == 1
    assert result['parallel_confirm_frames'] == 1


def test_multiple_physical_tags_with_the_same_id_are_ambiguous():
    monitor = AlignmentMonitor(2880, 720, 2160, confirm_frames=1)
    rows = monitor.observe([detection(), detection(x=2160)], 0.)
    assert all(row['observation_status'] == 'multiple_same_id' for row in rows)
    assert all(not row['horizontal_centered'] and not row['parallel_aligned'] for row in rows)


def test_unreliable_pose_does_not_block_bearing_but_cannot_confirm_parallel():
    monitor = AlignmentMonitor(2880, 720, 2160, confirm_frames=1)
    row = dict(detection(), pose_status='ambiguous_pose')
    result = monitor.observe([row], 0.)[0]
    assert result['horizontal_centered']
    assert not result['parallel_aligned']


def projected_quad(yaw_deg=25., depth=4., size=960):
    angle = np.deg2rad(yaw_deg)
    rotation = np.array([[np.cos(angle), 0, np.sin(angle)], [0, 1, 0],
                         [-np.sin(angle), 0, np.cos(angle)]]) @ np.diag([1., -1., -1.])
    rvec, _ = cv2.Rodrigues(rotation)
    square = np.array([[-.5, .5, 0], [.5, .5, 0], [.5, -.5, 0], [-.5, -.5, 0]])
    intrinsics = np.array([[size / 2, 0, (size - 1) / 2],
                           [0, size / 2, (size - 1) / 2], [0, 0, 1.]])
    corners, _ = cv2.projectPoints(square, rvec, np.array([.2, .1, depth]), intrinsics, None)
    return corners.reshape(4, 2)


@pytest.mark.parametrize('printed_rotation', [0, 1, 2, 3])
def test_plane_heading_recovers_known_yaw_in_a_side_facing_view(printed_rotation):
    corners = np.roll(projected_quad(), printed_rotation, axis=0)
    result = estimate_plane(corners, 960, 1440, 2880, 720, 3.)
    assert result['pose_status'] == 'ok'
    assert result['parallel_error_deg'] == pytest.approx(25., abs=1e-5)
    assert result['reprojection_error_px'] < 1e-5


def test_perspective_center_is_projected_tag_origin_not_corner_average():
    corners = projected_quad()
    expected = np.array([479.5 + 480 * .2 / 4, 479.5 + 480 * .1 / 4])
    assert marker_center(corners) == pytest.approx(expected)
    assert np.linalg.norm(corners.mean(axis=0) - expected) > 1.


def test_small_marker_cannot_produce_a_confident_plane_heading():
    assert estimate_plane(projected_quad(depth=100), 960, 1440, 2880, 720, 3.) == {
        'pose_status': 'tag_too_small'}


def test_known_tag_size_recovers_metric_position_in_robot_axis_convention():
    result = estimate_plane(projected_quad(), 960, 1440, 2880, 720, 3.,
                            tag_size_m=.2, camera_offset_xy=(.1, .05))
    assert result['pose_status'] == 'ok'
    assert result['range_m'] == pytest.approx(np.linalg.norm([.2, .1, 4.]) * .2)
    assert result['position_body_m'] == pytest.approx([.1 - .04, .05 - .8, -.02])
    assert np.linalg.norm(result['normal_body']) == pytest.approx(1.)


def test_camera_mount_rotation_and_height_transform_both_position_and_plane():
    roll, pitch = np.deg2rad([15., 10.])
    cr, sr, cp, sp = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch)
    rotation = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]) @ np.array([
        [1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    baseline = estimate_plane(projected_quad(), 960, 1440, 2880, 720, 3., tag_size_m=.2)
    result = estimate_plane(projected_quad(), 960, 1440, 2880, 720, 3., tag_size_m=.2,
                            camera_z_m=.4, camera_roll_pitch_rad=(roll, pitch))
    assert result['position_body_m'] == pytest.approx(rotation @ np.array(baseline['position_body_m']) + [0, 0, .4])
    assert result['normal_body'] == pytest.approx(rotation @ np.array(baseline['normal_body']))
    assert result['range_m'] == pytest.approx(baseline['range_m'])


def test_ubuntu_legacy_aruco_api_is_supported(monkeypatch):
    from types import SimpleNamespace
    aruco = cv2.aruco
    frame = synthetic_panorama(144)
    legacy = SimpleNamespace(getPredefinedDictionary=aruco.getPredefinedDictionary,
        DICT_APRILTAG_36h11=aruco.DICT_APRILTAG_36h11,
        CORNER_REFINE_SUBPIX=aruco.CORNER_REFINE_SUBPIX,
        DetectorParameters_create=aruco.DetectorParameters,
        detectMarkers=lambda image, dictionary, parameters:
            aruco.ArucoDetector(dictionary, parameters).detectMarkers(image))
    monkeypatch.setattr(cv2, 'aruco', legacy)
    detector = PanoramaDetector(576, 288, 144, view_size=256)
    assert detector.detect(frame)[0]['tag_id'] == 7


def test_two_distinct_low_error_plane_solutions_are_not_reported_as_aligned():
    result = estimate_plane(projected_quad(depth=20), 960, 1440, 2880, 720, 3.)
    assert result['pose_status'] == 'ambiguous_pose'
    assert 'parallel_error_deg' not in result


def synthetic_panorama(tag_x, width=576, tag_id=7):
    """Render a planar marker through rays into an ERP, including across its seam."""
    view = np.full((800, 800), 255, np.uint8)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    view[300:500, 300:500] = cv2.aruco.generateImageMarker(dictionary, tag_id, 200)
    y, x = np.mgrid[:width // 2, :width]
    longitude = (x - tag_x) * (2 * np.pi / width)
    latitude = (.5 - (y + .5) / (width // 2)) * np.pi
    z = np.cos(latitude) * np.cos(longitude)
    safe_z = np.where(np.abs(z) < 1e-6, 1e-6, z)
    map_x = (399.5 + 400 * np.cos(latitude) * np.sin(longitude) / safe_z).astype(np.float32)
    map_y = (399.5 - 400 * np.sin(latitude) / safe_z).astype(np.float32)
    image = cv2.remap(view, map_x, map_y, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT,
                      borderValue=255)
    image[z <= 0] = 255
    return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)


@pytest.mark.parametrize('tag_x', [0, 144, 288, 432])
def test_real_detector_recovers_rendered_tag_across_views_and_seam(tag_x):
    detector = PanoramaDetector(576, 288, 144, view_size=256)
    tags = detector.detect(synthetic_panorama(tag_x))
    assert len(tags) == 1  # Overlapping views must not count as separate tags.
    assert tags[0]['tag_id'] == 7
    assert abs(bearing_error(tags[0]['panorama_x'], tag_x, 576)) < 1.


def test_video_command_outputs_distinct_frames_config_and_preview(tmp_path, capsys):
    source = tmp_path / 'source.mp4'
    writer = cv2.VideoWriter(str(source), cv2.VideoWriter_fourcc(*'mp4v'), 5, (576, 288))
    assert writer.isOpened()
    for _ in range(5):
        writer.write(synthetic_panorama(288))
    writer.release()
    output, preview = tmp_path / 'results.jsonl', tmp_path / 'preview.mp4'
    main(['--video', str(source), '--front-center-x', '144', '--rear-center-x', '432',
          '--view-size', '256', '--output', str(output), '--preview', str(preview)])
    rows = [json.loads(line) for line in output.read_text().splitlines()]
    assert rows[0]['axes_calibration'] == 'provisional'
    assert rows[0]['physical_extrinsics_calibrated'] is False
    assert len(rows[1:]) == 5
    assert [r['frame_index'] for r in rows[1:]] == list(range(5))
    assert all(r['detections'][0]['tag_id'] == 7 for r in rows[1:])
    # Compression/sampling of a frontal small marker leaves two plausible
    # normals. Detection succeeds, while the plane heading stays unconfirmed.
    assert rows[-1]['detections'][0]['pose_status'] == 'ambiguous_pose'
    assert not rows[-1]['detections'][0]['parallel_aligned']
    assert preview.stat().st_size > 0
    assert 'sampled_frames=5 frames_with_tags=5' in capsys.readouterr().err
    with pytest.raises(SystemExit):
        main(['--video', str(source), '--output', str(source)])


def test_preview_keeps_missing_source_time_visible_without_repeating_tag_observations():
    from types import SimpleNamespace

    frames = []
    writer = SimpleNamespace(write=lambda frame: frames.append(frame.copy()))
    real = np.full((288, 576, 3), 200, np.uint8)
    next_slot = 0
    for stamp in (80., 80.2, 80.4, 81.8):
        next_slot = write_preview_sample(writer, real, stamp, 80., 5., next_slot)
    assert len(frames) == 10  # 2 seconds, including six unobserved slots.
    assert [i for i, frame in enumerate(frames) if frame[-1, -1, 0] == 200] == [0, 1, 2, 9]

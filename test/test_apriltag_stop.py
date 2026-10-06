import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

from types import SimpleNamespace

import numpy as np

from apriltag_stop import (
    AprilTagGate,
    AprilTagPolicy,
    AprilTagStopMonitor,
    CameraModel,
    camera_model_from_info,
    gate_detections,
    parse_tag_ids,
    tag_range_m,
)


def monitor():
    return AprilTagStopMonitor(
        AprilTagPolicy(max_age_sec=1.0, confirm_window_sec=1.0, min_hits=3)
    )


def test_empty_detection_is_a_heartbeat_not_a_candidate():
    tags = monitor()
    result = tags.observe(ids=[], frame_key=(1, 0), now=10.0, task_active=True)
    assert result.state == "no_tag"
    assert not result.stop_now
    assert tags.message_age_sec(10.25) == 0.25
    assert tags.stream_ready(10.99)
    assert not tags.stream_ready(11.01)


def test_first_candidate_stops_and_duplicate_frame_does_not_count_twice():
    tags = monitor()
    first = tags.observe(ids=[7, 7], frame_key=(2, 0), now=0.0, task_active=True)
    duplicate = tags.observe(ids=[7], frame_key=(2, 0), now=0.1, task_active=True)
    assert first.state == "verifying"
    assert first.stop_now
    assert dict(first.hit_counts) == {7: 1}
    assert not duplicate.stop_now
    assert dict(duplicate.hit_counts) == {7: 1}


def test_different_ids_do_not_combine_hits():
    tags = monitor()
    tags.observe(ids=[1], frame_key=1, now=0.0, task_active=True)
    tags.observe(ids=[2], frame_key=2, now=0.2, task_active=True)
    tags.observe(ids=[3], frame_key=3, now=0.4, task_active=True)
    result = tags.observe(ids=[], frame_key=4, now=1.0, task_active=True)
    assert result.state == "no_tag"
    assert result.false_positive
    assert result.confirmed_id is None


def test_inactive_task_records_heartbeat_without_starting_confirmation():
    tags = monitor()
    result = tags.observe(ids=[7], frame_key=1, now=0.0, task_active=False)
    assert result.state == "no_tag"
    assert not result.stop_now
    assert tags.stream_ready(0.0)


def test_nonempty_message_after_failed_window_starts_next_window_without_gap():
    tags = monitor()
    tags.observe(ids=[7], frame_key=1, now=0.0, task_active=True)
    result = tags.observe(ids=[7], frame_key=2, now=1.0, task_active=True)
    assert result.state == "verifying"
    assert result.false_positive
    assert result.stop_now
    assert dict(result.hit_counts) == {7: 1}


def test_same_id_three_frames_confirms_only_after_full_window():
    tags = monitor()
    tags.observe(ids=[7], frame_key=1, now=0.0, task_active=True)
    tags.observe(ids=[7], frame_key=2, now=0.1, task_active=True)
    early = tags.observe(ids=[7], frame_key=3, now=0.2, task_active=True)
    assert early.state == "verifying"
    assert tags.tick(now=0.99, task_active=True).state == "verifying"
    confirmed = tags.tick(now=1.0, task_active=True)
    assert confirmed.state == "confirmed"
    assert confirmed.just_confirmed
    assert confirmed.confirmed_id == 7
    assert (
        tags.observe(ids=[], frame_key=4, now=1.1, task_active=True).state
        == "confirmed"
    )


def test_timer_releases_unconfirmed_candidate_without_another_detection_message():
    tags = monitor()
    tags.observe(ids=[7], frame_key=1, now=0.0, task_active=True)
    result = tags.tick(now=1.0, task_active=True)
    assert result.state == "no_tag"
    assert result.false_positive
    assert not result.stop_now


def test_reset_task_clears_latch_but_preserves_fresh_heartbeat():
    tags = monitor()
    for frame, now in ((1, 0.0), (2, 0.1), (3, 0.2)):
        tags.observe(ids=[7], frame_key=frame, now=now, task_active=True)
    tags.tick(now=1.0, task_active=True)
    tags.reset_task()
    assert tags.snapshot(now=1.0).state == "no_tag"
    assert tags.stream_ready(1.0)


def test_begin_task_seeds_fresh_candidate_without_refreshing_heartbeat():
    tags = monitor()
    tags.observe(ids=[7], frame_key=9, now=10.0, task_active=False)
    seeded = tags.begin_task(ids=[7], frame_key=9, now=10.5)
    assert seeded.stop_now
    assert seeded.state == "verifying"
    assert seeded.window_elapsed_sec == 0.0
    assert dict(seeded.hit_counts) == {7: 1}
    assert tags.message_age_sec(10.75) == 0.75
    duplicate = tags.observe(ids=[7], frame_key=9, now=10.6, task_active=True)
    assert dict(duplicate.hit_counts) == {7: 1}
    tags.observe(ids=[7], frame_key=10, now=10.7, task_active=True)
    tags.observe(ids=[7], frame_key=11, now=10.8, task_active=True)
    assert tags.tick(now=11.49, task_active=True).state == "verifying"
    assert tags.tick(now=11.5, task_active=True).just_confirmed


@pytest.mark.parametrize(("ids", "now"), [([7], 11.01), ([], 10.5)])
def test_begin_task_does_not_seed_stale_or_empty_cache(ids, now):
    tags = monitor()
    tags.observe(ids=[7], frame_key=9, now=10.0, task_active=True)
    result = tags.begin_task(ids=ids, frame_key=9, now=now)
    assert result.state == "no_tag"
    assert not result.stop_now
    assert not result.hit_counts
    assert tags.message_age_sec(now) == pytest.approx(now - 10.0)


@pytest.mark.parametrize(
    "policy",
    [
        AprilTagPolicy(max_age_sec=0.0),
        AprilTagPolicy(max_age_sec=float("inf")),
        AprilTagPolicy(confirm_window_sec=float("nan")),
        AprilTagPolicy(min_hits=0),
        AprilTagPolicy(min_hits=True),
    ],
)
def test_policy_rejects_nonpositive_nonfinite_or_boolean_values(policy):
    with pytest.raises(ValueError):
        AprilTagStopMonitor(policy)


# Live /a2/front_camera/res_720p/camera_info, 2026-10-06.
A2_INFO = SimpleNamespace(
    k=(535.088326, 0.0, 643.406605, 0.0, 532.843368, 355.973065, 0.0, 0.0, 1.0),
    d=(-2.13869731, 0.77162026, 0.00048845, -0.001401,
       0.42753978, -1.85354262, 0.07940282, 0.84793409),
    distortion_model="rational_polynomial",
)
A2 = camera_model_from_info(A2_INFO)


def project(distance_m, *, yaw_deg=0.0, lateral_m=0.0, size_m=0.167):
    import cv2

    half = size_m / 2.0
    square = np.array(
        [[-half, half, 0.0], [half, half, 0.0], [half, -half, 0.0], [-half, -half, 0.0]]
    )
    rvec = np.array([0.0, np.radians(yaw_deg), 0.0])
    forward = np.sqrt(distance_m**2 - lateral_m**2)
    pixels, _ = cv2.projectPoints(
        square, rvec, np.array([lateral_m, 0.0, forward]),
        np.array(A2_INFO.k).reshape(3, 3), np.array(A2_INFO.d),
    )
    return [tuple(point) for point in pixels.reshape(4, 2)]


def detection(tag_id, distance_m, *, margin=50.0, **pose):
    return SimpleNamespace(
        id=tag_id,
        decision_margin=margin,
        corners=[SimpleNamespace(x=u, y=v) for u, v in project(distance_m, **pose)],
    )


def test_a2_calibration_is_usable_and_unknown_models_are_not():
    assert A2 == CameraModel(A2_INFO.k, A2_INFO.d, False)
    assert camera_model_from_info(SimpleNamespace(k=A2_INFO.k, d=(), distortion_model="")) is None
    assert camera_model_from_info(SimpleNamespace(k=(0.0,) * 9, d=A2_INFO.d,
                                                  distortion_model="rational_polynomial")) is None


@pytest.mark.parametrize("distance", [0.8, 2.0, 3.0, 4.5, 6.0])
@pytest.mark.parametrize("pose", [{}, {"yaw_deg": 35.0}, {"lateral_m": 0.6}])
def test_range_recovers_distance_through_the_wide_angle_distortion(distance, pose):
    assert tag_range_m(project(distance, **pose), 0.167, A2) == pytest.approx(
        distance, rel=0.01
    )


def test_range_ignores_corner_winding_and_rejects_bad_corners():
    corners = project(2.5)
    assert tag_range_m(corners[::-1], 0.167, A2) == pytest.approx(2.5, rel=0.01)
    assert tag_range_m(corners[:3], 0.167, A2) is None
    assert tag_range_m([(float("nan"), 0.0)] * 4, 0.167, A2) is None


def test_parse_tag_ids():
    assert parse_tag_ids("") == frozenset()
    assert parse_tag_ids("0-3, 7,9") == frozenset({0, 1, 2, 3, 7, 9})
    with pytest.raises(ValueError):
        parse_tag_ids("5-2")


def test_gate_keeps_only_tags_tag_approach_could_take_over():
    gate = AprilTagGate(max_range_m=3.0, min_decision_margin=15.0,
                        allowed_ids=parse_tag_ids("0-30"))
    gate.validate()
    accepted = gate_detections(
        [
            detection(1, 2.9),
            detection(2, 3.2),               # beyond range
            detection(3, 1.5, margin=10.0),  # faint
            detection(31, 1.5),              # not a tank tag
            detection(1, 1.2),               # nearer duplicate wins
        ],
        gate,
        A2,
    )
    assert set(accepted) == {1}
    assert accepted[1] == pytest.approx(1.2, rel=0.01)


def test_gate_without_calibration_accepts_nothing_unless_range_is_disabled():
    near = [detection(4, 1.0)]
    assert gate_detections(near, AprilTagGate(), None) == {}
    assert gate_detections(near, AprilTagGate(max_range_m=0.0), None) == {4: None}


def test_gate_rejects_invalid_limits():
    for gate in (AprilTagGate(max_range_m=-1.0), AprilTagGate(tag_size_m=0.0),
                 AprilTagGate(min_decision_margin=float("nan"))):
        with pytest.raises(ValueError):
            gate.validate()

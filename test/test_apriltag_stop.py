import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

from apriltag_stop import AprilTagPolicy, AprilTagStopMonitor


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

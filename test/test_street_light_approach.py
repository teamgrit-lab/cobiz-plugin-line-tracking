"""Scenario behavior: fresh evidence, lateral motion, and sequential visits."""

from dataclasses import replace
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from street_light_approach import (
    StreetLightApproach, StreetLightConfig, StreetLightObservation, street_light_observations,
)
from swin_l_drive_control import DriveDecision


NORMAL = DriveDecision(0.5, 0.0, 0.1, "tracking")
LIGHT = StreetLightObservation(0.3, 0.35, 0.25, 100)


def decide(controller, frame, observations=(LIGHT,), *, now=None, age=0.1,
           bearing=0.4, distance=5.0, normal=NORMAL):
    return controller.decide(
        normal, observations, now=frame / 4 if now is None else now,
        inference_id=frame, inference_age_sec=age,
        locate=lambda _: (bearing, distance),
    )


def confirm(controller, **kwargs):
    for frame in range(1, 4):
        result = decide(controller, frame, **kwargs)
    return result


@pytest.mark.parametrize("bearing", [-0.4, 0.4])
def test_confirmed_light_uses_lateral_motion_with_zero_yaw(bearing):
    controller = StreetLightApproach(StreetLightConfig())
    result = confirm(controller, bearing=bearing)
    assert result.reason == "street_light_approaching"
    assert result.vy * bearing > 0
    assert result.yaw_rate == 0
    assert result.vx == NORMAL.vx
    assert abs(result.vy) <= 0.15


def test_control_timer_cannot_confirm_a_single_model_detection():
    controller = StreetLightApproach(StreetLightConfig())
    for tick in range(100):
        assert decide(controller, 1, now=tick / 100).reason == "tracking"
    assert controller.hits == 1
    decide(controller, 3)  # A skipped inference cannot extend a consecutive run.
    assert controller.hits == 1


@pytest.mark.parametrize("reason", ["tracking", "tracking_slow_turn"])
def test_absent_or_unconfirmed_light_preserves_road_speed_and_steering(reason):
    controller = StreetLightApproach(StreetLightConfig())
    normal = replace(NORMAL, reason=reason, vx=0.25, yaw_rate=-0.12)
    assert decide(controller, 1, (), normal=normal) is normal
    assert decide(controller, 2, normal=normal) is normal
    assert decide(controller, 3, normal=normal) is normal
    biased = decide(controller, 4, normal=normal)
    assert biased.vx == normal.vx
    assert biased.vy > 0 and biased.yaw_rate == 0


@pytest.mark.parametrize("bearing,distance", [
    (float("nan"), None), (0.3, float("inf")), (0.3, -1),
])
def test_invalid_light_geometry_preserves_road_tracking(bearing, distance):
    controller = StreetLightApproach(StreetLightConfig())
    assert confirm(controller, bearing=bearing, distance=distance) == NORMAL
    assert controller.metrics["bias_active"] is False


def test_light_location_failure_preserves_road_tracking():
    controller = StreetLightApproach(StreetLightConfig())
    confirm(controller)

    def unavailable(_observation):
        raise ValueError("street_light_geometry_invalid")

    result = controller.decide(NORMAL, (LIGHT,), now=1, inference_id=4,
                               inference_age_sec=0.1, locate=unavailable)
    assert result is NORMAL
    assert controller.metrics["bias_active"] is False


def test_centered_light_leaves_road_steering_in_control():
    controller = StreetLightApproach(StreetLightConfig())
    assert confirm(controller, bearing=0.02) == NORMAL
    assert controller.metrics["bias_active"] is False


def test_visited_light_stays_excluded_after_occlusion_until_task_reset():
    controller = StreetLightApproach(StreetLightConfig())
    confirm(controller)
    result = decide(controller, 4, distance=0.95)
    assert result == NORMAL
    assert controller.visits == 1
    for frame in range(5, 15):
        result = decide(controller, frame, distance=0.95)
        assert result.reason == "tracking"
        assert result == NORMAL
    for frame in range(15, 18):
        decide(controller, frame, ())
    assert controller.phase == "cruising"
    for frame in range(18, 21):
        result = decide(controller, frame)
    assert result.reason == "tracking"
    assert controller.metrics["visible_ids"] == controller.metrics["visited_ids"] == [1]
    assert controller.metrics["pending_ids"] == []
    assert controller.visits == 1
    controller.reset()
    assert confirm(controller).reason == "street_light_approaching"
    assert controller.visits == 0
    assert controller.metrics["visited_ids"] == []


def test_continuously_visible_lights_are_visited_in_order_despite_size_and_order_changes():
    controller = StreetLightApproach(StreetLightConfig())
    first = replace(LIGHT, area_px=300)
    second = StreetLightObservation(0.6, 0.65, 0.25, 100)
    third = StreetLightObservation(0.85, 0.9, 0.25, 2000)
    confirm(controller, observations=(second, first))
    assert controller.target == first
    assert controller.metrics["pending_ids"] == [2]
    # A larger new lamp must not replace the locked target or jump the queue.
    second = replace(second, area_px=500)
    assert decide(controller, 4, (third, second, first), distance=0.9) == NORMAL
    assert controller.metrics["visited_ids"] == [1]
    assert controller.metrics["pending_ids"] == [2, 3]
    assert decide(controller, 4, (third, second, first)).reason == "tracking"
    assert decide(controller, 5, (first, third, second)).reason == "street_light_approaching"
    assert controller.target == second
    assert controller.target_id == 2
    assert decide(controller, 6, (first, second, third), distance=0.9) == NORMAL
    assert decide(controller, 7, (third, second, first)).reason == "street_light_approaching"
    assert controller.target_id == 3
    assert decide(controller, 8, (first, second, third), distance=0.9) == NORMAL
    for frame in range(9, 15):
        assert decide(controller, frame, (third, first, second)).reason == "tracking"
    assert controller.visits == 3
    assert controller.metrics["visited_ids"] == [1, 2, 3]


def test_close_lights_have_distinct_ids_and_visited_light_can_reappear_during_next_approach():
    controller = StreetLightApproach(StreetLightConfig())
    # Both centers lie within the old single-target matching window (0.15).
    first = replace(LIGHT, area_px=300)
    second = StreetLightObservation(0.4, 0.45, 0.25, 100)
    confirm(controller, observations=(first, second))
    decide(controller, 4, (first, second), distance=0.9)
    for frame in range(5, 10):
        assert decide(controller, frame, (second,)).reason == "street_light_approaching"
        assert controller.target_id == 2
    assert decide(controller, 10, (second, first)).reason == "street_light_approaching"
    assert controller.target_id == 2
    assert controller.metrics["visible_ids"] == [1, 2]
    assert controller.metrics["visited_ids"] == [1]
    assert len(controller.tracks) == 2


def test_track_ids_follow_moving_lights_without_reselecting_the_largest_component():
    controller = StreetLightApproach(StreetLightConfig())
    first = replace(LIGHT, area_px=300)
    second = StreetLightObservation(0.5, 0.55, 0.25, 100)
    confirm(controller, observations=(first, second))
    decide(controller, 4, (first, second), distance=0.9)
    for frame in range(5, 12):
        first = replace(first, left=first.left + 0.04, right=first.right + 0.04)
        second = replace(second, left=second.left + 0.04, right=second.right + 0.04)
        assert decide(controller, frame, (second, first)).reason == "street_light_approaching"
        assert controller.target_id == 2
        assert controller.target == second
        assert controller.metrics["visited_ids"] == [1]
    assert len(controller.tracks) == 2


def test_merged_components_stop_approach_without_creating_duplicate_targets():
    controller = StreetLightApproach(StreetLightConfig())
    first = replace(LIGHT, area_px=300)
    second = StreetLightObservation(0.4, 0.45, 0.25, 100)
    confirm(controller, observations=(first, second))
    decide(controller, 4, (first, second), distance=0.9)
    decide(controller, 5, (first, second))
    merged = StreetLightObservation(0.3, 0.45, 0.25, 450)
    assert decide(controller, 6, (merged,)) == NORMAL
    for frame in range(7, 12):
        assert decide(controller, frame, (merged,)).reason == "tracking"
    for frame in range(12, 15):
        result = decide(controller, frame, (second, first))
    assert result.reason == "street_light_approaching"
    assert controller.target_id == 2
    assert controller.metrics["visited_ids"] == [1]
    assert len(controller.tracks) == 2


@pytest.mark.parametrize("failure", ["loss", "timeout"])
def test_failed_target_blocks_only_itself_and_other_visible_lights_still_run(failure):
    controller = StreetLightApproach(StreetLightConfig(max_approach_sec=2))
    first = replace(LIGHT, area_px=300)
    second = StreetLightObservation(0.7, 0.75, 0.25, 100)
    confirm(controller, observations=(first, second))
    visible = (second,) if failure == "loss" else (first, second)
    result = decide(controller, 4, visible, now=3.0)
    assert result == NORMAL
    assert controller.metrics["transition"] == ("target_lost" if failure == "loss" else "approach_timeout")
    assert controller.metrics["visited_ids"] == []
    assert controller.tracks[1].retry_blocked
    assert decide(controller, 5, (first, second), now=3.1).reason == "street_light_approaching"
    assert controller.target_id == 2
    assert controller.visits == 0


def test_failed_target_needs_its_own_consecutive_absence_before_retry():
    controller = StreetLightApproach(StreetLightConfig(max_approach_sec=2))
    confirm(controller)
    decide(controller, 4, now=3.0)
    for frame in range(5, 10):
        assert decide(controller, frame, now=3 + frame / 10).reason == "tracking"
    decide(controller, 10, ())
    decide(controller, 12, ())  # Skipped inference restarts the absence count.
    assert controller.tracks[1].retry_blocked
    decide(controller, 13, ())
    assert controller.tracks[1].retry_blocked
    decide(controller, 14, ())
    assert not controller.tracks[1].retry_blocked
    for frame in range(15, 18):
        result = decide(controller, frame)
    assert result.reason == "street_light_approaching"
    assert controller.target_id == 1


def test_next_light_requires_its_own_fresh_confirmations():
    controller = StreetLightApproach(StreetLightConfig())
    confirm(controller)
    second = StreetLightObservation(0.7, 0.75, 0.25, 100)
    decide(controller, 4, (LIGHT, second), distance=0.9)
    assert decide(controller, 5, (LIGHT, second)).reason == "tracking"
    for _ in range(5):
        assert decide(controller, 5, (LIGHT, second)).reason == "tracking"
    assert decide(controller, 7, (LIGHT, second)).reason == "tracking"
    assert controller.hits == 1
    decide(controller, 8, (LIGHT, second))
    assert decide(controller, 9, (LIGHT, second)).reason == "street_light_approaching"
    assert controller.target_id == 2


def test_target_loss_resumes_road_without_retaining_lateral_velocity():
    controller = StreetLightApproach(StreetLightConfig())
    confirm(controller)
    result = decide(controller, 4, ())
    assert result == NORMAL
    assert controller.phase == "passing"
    assert controller.metrics["transition"] == "target_lost"
    assert decide(controller, 4, ()).reason == "tracking"


def test_never_switches_to_an_unrelated_light_during_an_approach():
    controller = StreetLightApproach(StreetLightConfig())
    confirm(controller)
    other = StreetLightObservation(0.8, 0.85, 0.25, 1000)
    result = decide(controller, 4, (other,))
    assert result == NORMAL
    assert controller.phase == "passing"


@pytest.mark.parametrize("age", [None, -0.01, 1.01, float("nan"), float("inf")])
def test_stale_light_evidence_disables_only_bias_and_preserves_road_tracking(age):
    controller = StreetLightApproach(StreetLightConfig())
    confirm(controller)
    assert decide(controller, 4, age=age) == NORMAL
    assert controller.metrics["bias_active"] is False
    assert controller.metrics["bias_reason"] == "observations_stale"
    # Old confirmations cannot resume a lateral command on the next fresh frame.
    assert decide(controller, 5) == NORMAL
    assert decide(controller, 6) == NORMAL
    assert decide(controller, 7).reason == "street_light_approaching"


def test_missing_lidar_match_never_enables_bias_or_invents_arrival():
    controller = StreetLightApproach(StreetLightConfig(max_approach_sec=2))
    assert confirm(controller, distance=None) == NORMAL
    for frame in range(4, 10):
        assert decide(controller, frame, now=frame, distance=None) == NORMAL
        assert controller.metrics["bias_reason"] == "lidar_unconfirmed"
        assert controller.metrics["lidar_confirmed"] is False
    assert controller.approach_started is None
    assert controller.visits == 0


@pytest.mark.parametrize("reason", ["branch_path_lost", "path_unavailable", "camera_stale"])
def test_drive_guards_precede_scenario_motion(reason):
    controller = StreetLightApproach(StreetLightConfig())
    confirm(controller)
    assert decide(controller, 4, normal=DriveDecision.stop(reason)) == DriveDecision.stop(reason)


def test_components_filter_noise_and_do_not_average_opposite_side_lights():
    mask = np.zeros((100, 200), bool)
    mask[10:20, 20:30] = True
    mask[10:20, 160:170] = True
    mask[60, 100] = True
    observations = street_light_observations(mask, 24)
    assert len(observations) == 2
    assert observations[0].center < 0.5 < observations[1].center
    assert observations[0].top == observations[1].top == .1


@pytest.mark.parametrize("field,value", [
    ("stop_distance_m", float("nan")), ("max_lateral_mps", 1.1),
    ("confirm_frames", 1), ("min_area_px", 0), ("max_age_sec", 0),
    ("min_confidence", float("nan")), ("min_confidence", 1.1),
    ("lidar_min_points", 0), ("lidar_max_range_m", float("nan")),
])
def test_invalid_scenario_configuration_fails_early(field, value):
    with pytest.raises(ValueError):
        replace(StreetLightConfig(), **{field: value}).validate()

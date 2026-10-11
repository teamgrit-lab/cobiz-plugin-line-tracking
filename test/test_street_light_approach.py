"""Scenario behavior: fresh evidence, lateral preference, visits, and rearming."""

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
           bearing=0.4, distance=None, normal=NORMAL):
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
    assert 0 < result.vx <= abs(result.vy) / 3
    assert abs(result.vy) <= 0.15


def test_control_timer_cannot_confirm_a_single_model_detection():
    controller = StreetLightApproach(StreetLightConfig())
    for tick in range(100):
        assert decide(controller, 1, now=tick / 100).reason == "street_light_cruising"
    assert controller.hits == 1
    decide(controller, 3)  # A skipped inference cannot extend a consecutive run.
    assert controller.hits == 1


def test_reached_light_resumes_cruising_and_is_not_chased_again_until_absent():
    controller = StreetLightApproach(StreetLightConfig())
    confirm(controller)
    result = decide(controller, 4, distance=0.95)
    assert result == DriveDecision.stop("street_light_reached")
    assert controller.visits == 1
    for frame in range(5, 15):
        result = decide(controller, frame, distance=0.95)
        assert result.reason == "street_light_cruising"
        assert result.vy == result.yaw_rate == 0
    for frame in range(15, 18):
        decide(controller, frame, ())
    assert controller.phase == "cruising"
    for frame in range(18, 21):
        result = decide(controller, frame)
    assert result.reason == "street_light_approaching"


def test_target_loss_resumes_road_without_retaining_lateral_velocity():
    controller = StreetLightApproach(StreetLightConfig())
    confirm(controller)
    result = decide(controller, 4, ())
    assert result.reason == "street_light_target_lost"
    assert result.vy == result.yaw_rate == 0
    assert controller.phase == "passing"
    assert controller.metrics["transition"] == "target_lost"
    assert decide(controller, 4, ()).reason == "street_light_cruising"


def test_never_switches_to_an_unrelated_light_during_an_approach():
    controller = StreetLightApproach(StreetLightConfig())
    confirm(controller)
    other = StreetLightObservation(0.8, 0.85, 0.25, 1000)
    result = decide(controller, 4, (other,))
    assert result.reason == "street_light_target_lost"
    assert controller.phase == "passing"


@pytest.mark.parametrize("age", [None, -0.01, 1.01, float("nan"), float("inf")])
def test_stale_inference_always_stops_approach_even_if_optional_stops_are_off(age):
    controller = StreetLightApproach(StreetLightConfig())
    confirm(controller)
    assert decide(controller, 4, age=age) == DriveDecision.stop("street_light_inference_stale")


def test_missing_lidar_range_does_not_invent_arrival_and_approach_is_bounded():
    controller = StreetLightApproach(StreetLightConfig(max_approach_sec=2))
    assert confirm(controller).reason == "street_light_approaching"
    result = decide(controller, 4, now=3.0)
    assert result == DriveDecision.stop("street_light_approach_timeout")
    assert decide(controller, 4, now=3.1).reason == "street_light_cruising"
    assert controller.metrics["transition"] == "approach_timeout"
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


@pytest.mark.parametrize("field,value", [
    ("stop_distance_m", float("nan")), ("max_lateral_mps", 1.1),
    ("confirm_frames", 1), ("min_area_px", 0), ("max_age_sec", 0),
    ("min_confidence", float("nan")), ("min_confidence", 1.1),
])
def test_invalid_scenario_configuration_fails_early(field, value):
    with pytest.raises(ValueError):
        replace(StreetLightConfig(), **{field: value}).validate()

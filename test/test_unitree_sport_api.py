from pathlib import Path
from types import SimpleNamespace
import sys

import pytest


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

from unitree_sport_api import (  # noqa: E402
    ROBOT_SPORT_API_ID_MOVE,
    ROBOT_SPORT_API_ID_STOP_MOVE,
    drive_to_sport_move,
    populate_move_request,
    populate_stop_move_request,
)


def test_drive_to_sport_move_preserves_field_calibrated_a2_signs():
    move = drive_to_sport_move(vx=0.10, vy=0.04, yaw_rate=0.08)

    assert (move.x, move.y, move.z) == pytest.approx((0.10, -0.04, -0.08))
    assert move.parameter_json() == '{"x":0.1,"y":-0.04,"z":-0.08}'


def test_zero_move_serializes_positive_exact_zeroes():
    move = drive_to_sport_move(vx=0.0, vy=0.0, yaw_rate=0.0)

    assert (move.x, move.y, move.z) == (0.0, 0.0, 0.0)
    assert move.parameter_json() == '{"x":0.0,"y":0.0,"z":0.0}'


@pytest.mark.parametrize("value", (float("nan"), float("inf"), -float("inf")))
def test_non_finite_drive_values_are_rejected(value):
    with pytest.raises(ValueError, match="finite"):
        drive_to_sport_move(vx=value, vy=0.0, yaw_rate=0.0)


def test_populate_move_request_sets_unitree_move_contract():
    request = SimpleNamespace(
        header=SimpleNamespace(identity=SimpleNamespace(api_id=0)),
        parameter="",
        binary=[],
    )
    move = drive_to_sport_move(vx=0.10, vy=0.04, yaw_rate=0.08)

    returned = populate_move_request(request, move)

    assert returned is request
    assert ROBOT_SPORT_API_ID_MOVE == 1008
    assert request.header.identity.api_id == 1008
    assert request.parameter == '{"x":0.1,"y":-0.04,"z":-0.08}'


def test_populate_stop_move_request_sets_unitree_contract():
    request = SimpleNamespace(
        header=SimpleNamespace(identity=SimpleNamespace(api_id=0)),
        parameter="",
        binary=[],
    )

    returned = populate_stop_move_request(request)

    assert returned is request
    assert ROBOT_SPORT_API_ID_STOP_MOVE == 1003
    assert request.header.identity.api_id == 1003
    assert request.parameter == "{}"

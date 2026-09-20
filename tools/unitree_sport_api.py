"""Unitree Sport Move request encoding without a ROS runtime dependency."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from typing import Any


ROBOT_SPORT_API_ID_MOVE = 1008
ROBOT_SPORT_API_ID_STOP_MOVE = 1003


def _finite_zero_normalized(value: float) -> float:
    converted = float(value)
    if not math.isfinite(converted):
        raise ValueError("A2 Sport Move values must be finite")
    return 0.0 if converted == 0.0 else converted


@dataclass(frozen=True)
class SportMove:
    """A Unitree Sport Move payload in the robot's x/y/yaw convention."""

    x: float
    y: float
    z: float

    def parameter_json(self) -> str:
        return json.dumps(
            {"x": self.x, "y": self.y, "z": self.z},
            separators=(",", ":"),
            allow_nan=False,
        )


def drive_to_sport_move(*, vx: float, vy: float, yaw_rate: float) -> SportMove:
    """Preserve the field-corrected signs formerly applied by a2_control."""

    return SportMove(
        x=_finite_zero_normalized(vx),
        y=_finite_zero_normalized(-vy),
        z=_finite_zero_normalized(-yaw_rate),
    )


def populate_move_request(request: Any, move: SportMove) -> Any:
    """Fill a generated ``unitree_api.msg.Request`` with one Move command."""

    request.header.identity.api_id = ROBOT_SPORT_API_ID_MOVE
    request.parameter = move.parameter_json()
    return request


def populate_stop_move_request(request: Any) -> Any:
    """Fill a generated request with one Unitree Sport StopMove command."""

    request.header.identity.api_id = ROBOT_SPORT_API_ID_STOP_MOVE
    request.parameter = "{}"
    return request

import numpy as np

from line_tracking.controller import ControllerConfig, LineTrackingController
from line_tracking.vision import PathEstimate


def _path(coefficients, confidence=0.9):
    return PathEstimate(
        coefficients=np.asarray(coefficients, dtype=np.float64),
        confidence=confidence,
        residual_m=0.01,
        points_xy=np.zeros((7, 2), dtype=np.float64),
    )


def test_generates_forward_and_left_command_for_left_offset():
    controller = LineTrackingController(ControllerConfig(filter_alpha=1.0))

    command = controller.generate(_path([0.0, 0.0, 0.10]), dt=1.0)

    assert 0.0 < command.vx <= 0.30
    assert command.vy > 0.0
    assert abs(command.yaw_rate) < 1e-9


def test_curvature_feed_forward_starts_positive_yaw():
    controller = LineTrackingController(ControllerConfig(filter_alpha=1.0))

    command = controller.generate(_path([0.12, 0.0, 0.0]), dt=1.0)

    assert command.curvature > 0.0
    assert command.heading_error > 0.0
    assert command.yaw_rate > 0.0


def test_low_confidence_stops_immediately():
    controller = LineTrackingController(ControllerConfig(filter_alpha=1.0))
    moving = controller.generate(_path([0.0, 0.0, 0.0], confidence=0.9), dt=1.0)
    stopped = controller.generate(_path([0.0, 0.0, 0.0], confidence=0.2), dt=0.01)

    assert moving.vx > 0.0
    assert stopped.vx == 0.0
    assert stopped.vy == 0.0
    assert stopped.yaw_rate == 0.0


def test_rate_limits_first_command():
    config = ControllerConfig(
        filter_alpha=1.0,
        max_vx_rate=0.30,
        max_vy_rate=0.25,
        max_yaw_rate_change=0.80,
    )
    controller = LineTrackingController(config)

    command = controller.generate(_path([0.2, 0.0, 0.2]), dt=0.1)

    assert command.vx <= 0.0300001
    assert abs(command.vy) <= 0.0250001
    assert abs(command.yaw_rate) <= 0.0800001

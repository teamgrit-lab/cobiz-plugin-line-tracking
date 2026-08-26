import pytest

from line_tracking.controller import VelocityCommand
from line_tracking.joy_command import command_to_joy_axes, released_buttons


def test_maps_velocity_to_a2_control_axes_with_required_signs():
    command = VelocityCommand(vx=0.25, vy=-0.08, yaw_rate=0.30)

    axes = command_to_joy_axes(command)

    assert axes == pytest.approx([0.08, -0.25, -0.30])
    assert -axes[1] == pytest.approx(command.vx)
    assert -axes[0] == pytest.approx(command.vy)
    assert -axes[2] == pytest.approx(command.yaw_rate)


def test_all_a2_control_buttons_are_released():
    buttons = released_buttons()

    assert len(buttons) == 10
    assert buttons == [0] * 10


def test_rejects_button_array_too_short_for_a2_contract():
    with pytest.raises(ValueError):
        released_buttons(9)

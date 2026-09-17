"""Convert robot-frame velocity commands to the cobiz-plugin-a2 Joy contract."""

from typing import List

from .controller import VelocityCommand


def command_to_joy_axes(command: VelocityCommand) -> List[float]:
    """Return axes for the A2 with field-corrected lateral and yaw polarity.

    a2_control_node applies vx=-axes[1], vy=-axes[0], yaw=-axes[2].
    Per the reported field behavior, physical lateral and yaw directions are
    reversed relative to this controller's path frame. Move gets -vy and
    -yaw_rate.
    """

    return [command.vy, -command.vx, command.yaw_rate]


def released_buttons(count: int = 10) -> List[int]:
    """Return a button array that cannot trigger A2 pose or gait actions."""

    if count < 10:
        raise ValueError("A2 Joy messages require at least 10 released buttons")
    return [0] * count

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

import pytest
from navigation_segment import NavigationSegment
from unitree_sport_api import drive_to_sport_move


def command(op="START", **extra):
    return dict(protocol=1, session_id="owner", segment_id="1", command=op,
                selected_mask=2, stamp_sec=100., lease_sec=1., **extra)


def test_duplicate_start_does_not_restart_engine_and_cannot_change_surface():
    segment = NavigationSegment()
    assert segment.handle(command(), now=0, ros_now=100)["effect"] == "START"
    assert segment.handle(command(), now=.1, ros_now=100.1) is None
    result = segment.handle({**command(), "selected_mask": 1}, now=.1, ros_now=100.1)
    assert result["state"] == "REJECTED"
    assert segment.active["selected_mask"] == 2


def test_stop_before_start_and_replayed_start_cannot_revive_cancelled_segment():
    segment = NavigationSegment()
    assert segment.handle(command("STOP"), now=0, ros_now=100)["state"] == "STOPPED"
    assert segment.handle(command(), now=0, ros_now=100)["state"] == "STOPPED"
    assert segment.active is None


def test_foreign_stop_does_not_clear_current_owner_and_lease_expires():
    segment = NavigationSegment()
    segment.handle(command(), now=0, ros_now=100)
    segment.handle({**command("STOP"), "session_id": "old"}, now=.1, ros_now=100.1)
    assert segment.active["session_id"] == "owner"
    assert segment.expired(1.01)
    identity = segment.close("lease expired", error=True)
    assert identity == {"session_id": "owner", "segment_id": "1"}
    assert segment.handle(command(), now=0, ros_now=100)["state"] == "ERROR"


@pytest.mark.parametrize("updates", [{"stamp_sec": 99.}, {"stamp_sec": float("nan")},
    {"protocol": True}, {"lease_sec": float("inf")}, {"lease_sec": 10}, {"selected_mask": True}])
def test_invalid_control_cannot_activate_engine(updates):
    segment = NavigationSegment()
    segment.handle({**command(), **updates}, now=0, ros_now=100)
    assert segment.active is None


def test_velocity_preserves_existing_sport_axis_signs_once_and_increments_sequence():
    segment = NavigationSegment()
    segment.handle(command(), now=0, ros_now=100)
    move = drive_to_sport_move(vx=.2, vy=.1, yaw_rate=-.1)
    first = segment.velocity(move, stamp=100.)
    second = segment.velocity(move, stamp=100.1)
    assert (first["vx"], first["vy"], first["yaw_rate"]) == (.2, -.1, -.1)
    assert second["sequence"] == first["sequence"] + 1


def test_stale_heartbeat_does_not_extend_lease():
    segment = NavigationSegment()
    segment.handle(command(), now=0, ros_now=100)
    segment.handle({**command("HEARTBEAT"), "stamp_sec": 99.9}, now=.9, ros_now=100.9)
    assert segment.expired(1.01)

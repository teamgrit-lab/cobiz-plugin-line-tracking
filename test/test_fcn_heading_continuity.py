"""Current corridor evidence, confirmation, and complete-loss behavior.

Controlled corridor observations isolate temporal decisions; the projection
and real semantic packet tests exercise the geometry producing these bounds.
"""
import numpy as np
import pytest

from dangjin_fcn import FcnDrive


def controller(monkeypatch, shape=(16, 32)):
    drive = FcnDrive(1.6, 2.)
    road = np.full((4, *shape), 3, np.uint8)
    observation = dict(slope=0., half_width=3.2)

    def bounds(_):
        center = drive.planner.x[:, None] * observation['slope']
        shape = drive.planner.rays.shape
        return (np.broadcast_to(center-observation['half_width'], shape).copy(),
                np.broadcast_to(center+observation['half_width'], shape).copy())

    monkeypatch.setattr(drive.planner, 'corridor_bounds', bounds)
    return drive, road, observation


@pytest.mark.parametrize('shape', [(8, 16), (16, 32)])
def test_centring_jitter_keeps_moving_without_turn_confirmation(monkeypatch, shape):
    drive, road, observation = controller(monkeypatch, shape)
    for i, slope in enumerate([.15, .20, .25, .30, .25, .20, .15, .30]):
        observation['slope'] = slope
        cmd, info, points = drive.command(road, 1.+i*.1, 2.)
        assert cmd.vx > 0 and cmd.vy == 0
        assert info['maneuver'] == 'straight'
        assert info['front_path_valid'] and info['turn_confirmation_frames'] == 0
        assert points


def test_turn_confirmation_rebuilds_previous_bearing_in_current_corridor(monkeypatch):
    drive, road, observation = controller(monkeypatch)
    _, _, original = drive.command(road, 1., 2.)
    observation['slope'] = .65
    for i in range(2):
        cmd, info, points = drive.command(road, 1.1+i*.1, 2.)
        assert 0 < cmd.vx <= .9 and cmd.reason == 'tracking_turn_confirmation'
        assert info['maneuver'] == 'confirm_left'
        assert info['turn_confirmation_frames'] == i+1
        assert info['path_source'] == 'revalidated_previous_bearing'
        assert info['front_path_valid'] and info['candidate_path_valid']
        points = np.asarray(points)
        # The old 8 m path no longer fits: fresh evidence truncates it.
        assert len(points) < len(original)
        center = observation['slope']*points[:, 0]
        assert np.all(abs(points[:, 1]-center) <= observation['half_width']-.65+1e-6)
    cmd, info, _ = drive.command(road, 1.3, 2.)
    assert cmd.vx > 0 and info['maneuver'] == 'turn_left'
    assert info['path_source'] == 'current_candidate'


def test_no_current_continuation_stops_with_distinct_confirmation_reason(monkeypatch):
    drive, road, observation = controller(monkeypatch)
    observation.update(slope=.9, half_width=1.3)
    for i in range(2):
        cmd, info, points = drive.command(road, 1.+i*.1, 2.)
        assert (cmd.vx, cmd.yaw_rate) == (0., 0.)
        assert cmd.reason == 'turn_confirmation'
        assert info['candidate_path_valid'] and not info['front_path_valid']
        assert info['candidate_horizon_m'] >= 3.2 and not points
    assert drive.command(road, 1.2, 2.)[0].vx > 0


def test_confirmed_turn_does_not_reconfirm_in_direction_dead_band(monkeypatch):
    drive, road, observation = controller(monkeypatch)
    for i, slope in enumerate([0., .45, .45, .45, .375, .425, .30, .375, .425]):
        observation['slope'] = slope
        cmd, info, _ = drive.command(road, 1.+i*.1, 2.)
        assert cmd.vx > 0
        if i >= 3:
            assert info['maneuver'] == 'turn_left'
            assert info['path_status'] == 'tracking'


def test_repeated_stamp_cannot_confirm_a_new_turn(monkeypatch):
    drive, road, observation = controller(monkeypatch)
    observation.update(slope=.9, half_width=1.3)
    for _ in range(5):
        assert drive.command(road, 1., 2.)[0].reason == 'turn_confirmation'
    assert drive.command(road, 1.1, 2.)[0].vx == 0
    assert drive.command(road, 1.2, 2.)[0].vx > 0


def test_full_loss_clears_motion_history_and_cannot_reuse_a_path(monkeypatch):
    drive, road, observation = controller(monkeypatch)
    assert drive.command(road, 1., 2.)[0].vx > 0
    # Full dropout must reject before the controlled geometry is consulted.
    for i in range(5):
        cmd, info, points = drive.command(np.zeros_like(road), 1.1+i*.1, 2.)
        assert (cmd.vx, cmd.yaw_rate) == (0., 0.) and not points
        assert not info['front_path_valid'] and not info['candidate_path_valid']
    observation.update(slope=.9, half_width=1.3)
    assert drive.command(road, 1.6, 2.)[0].reason == 'turn_confirmation'

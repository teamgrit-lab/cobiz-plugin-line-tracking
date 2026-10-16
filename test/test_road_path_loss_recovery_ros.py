"""Road section execution and path-loss search share fresh confirmation safely."""
import json

import pytest

from test_apriltag_task_stop_ros import SPORT, ZERO, debug
from test_path_loss_recovery_ros import run_recovery


def road_recovery(monkeypatch, scenario, *, execution=True, confirm_frames=2):
    monkeypatch.setitem(debug.ENV, 'LINE_TRACKING_MAX_FORWARD_MPS', '.8')
    run_recovery(monkeypatch, scenario, mask_class=1, extra_args=(
        '--path-execution-enabled' if execution else '--no-path-execution-enabled',
        '--path-execution-confirm-frames', str(confirm_frames)))


@pytest.mark.parametrize('hz', [1, 2])
def test_road_stops_on_loss_then_waits_scans_and_reacquires_at_low_inference_hz(monkeypatch, hz):
    def scenario(ros, node, feed):
        def frame(**kwargs):
            # 10 Hz output continues between the 1/2 Hz completed inferences.
            for _ in range(round(10/hz)-1):
                ros.now += .1
                node.publish_state()
            return feed(**kwargs)

        for _ in range(4):
            result = feed()
            assert not result['path_recovery']['active']  # No search at startup.
            assert json.loads(ros.published[SPORT][-1].parameter) == ZERO
        first = feed(visible=True)
        assert first['drive_reason'] == 'path_execution_waiting_initial_candidate'
        assert not first['path_recovery']['active']
        assert frame(visible=True)['drive_reason'] == 'tracking'
        assert json.loads(ros.published[SPORT][-1].parameter)['x'] == .8

        for count in (1, 2):
            result = frame()
            assert result['drive_reason'] == 'road_path_unconfirmed'
            assert result['path_recovery']['failed_inferences'] == count
            assert not result['path_recovery']['active']
            assert json.loads(ros.published[SPORT][-1].parameter) == ZERO
        result = frame()
        assert result['drive_reason'] == 'path_recovery_waiting'
        assert result['path_recovery']['failed_inferences'] == 3
        for _ in range(10):
            node.publish_state()
        assert ros.metrics()['path_recovery']['failed_inferences'] == 3
        for _ in range(4):
            result = frame()
            if result['drive_reason'] == 'path_recovery_scan_left':
                break
        assert result['drive_reason'] == 'path_recovery_scan_left'
        move = json.loads(ros.published[SPORT][-1].parameter)
        assert move['x'] == move['y'] == 0 and 0 < move['z'] <= .18

        result = frame(visible=True)
        assert result['drive_reason'] == 'path_recovery_confirming'
        assert result['path_recovery']['confirmed_frames'] == 1
        assert json.loads(ros.published[SPORT][-1].parameter) == ZERO
        for _ in range(10):
            node.publish_state()
        assert ros.metrics()['path_recovery']['confirmed_frames'] == 1
        result = frame(visible=True)
        assert result['drive_reason'] == 'tracking'
        assert not result['path_recovery']['active']
        move = json.loads(ros.published[SPORT][-1].parameter)
        assert move['x'] == .8 and abs(move['y']) <= .25

        # A separate later loss receives a new waiting window.
        for _ in range(3):
            result = frame()
        assert result['drive_reason'] == 'path_recovery_waiting'
        node.release_task_control('test_cancel')
        node.publish_state()
        assert json.loads(ros.published[SPORT][-1].parameter) == ZERO
        assert not ros.metrics()['path_recovery']['active']
    road_recovery(monkeypatch, scenario)


@pytest.mark.parametrize('during_search', [False, True])
def test_road_positive_blockage_stops_and_cancels_search(monkeypatch, during_search):
    def scenario(ros, node, feed):
        feed(visible=True)
        assert feed(visible=True)['drive_reason'] == 'tracking'
        if during_search:
            for _ in range(15):
                result = feed()
            assert result['path_recovery']['active']
        for _ in range(10):
            result = feed(visible=True, blocked=True)
            assert result['drive_reason'] == 'road_clearance_blocked'
            assert result['road_clearance']['blocked']
            assert not result['path_recovery']['active']
            assert result['path_recovery']['failed_inferences'] == 0
            assert json.loads(ros.published[SPORT][-1].parameter) == ZERO
        assert feed(visible=True)['drive_reason'] == 'path_execution_invalidated'
        assert feed(visible=True)['drive_reason'] == 'tracking'
    road_recovery(monkeypatch, scenario)


def test_road_recovery_disabled_keeps_execution_stop(monkeypatch):
    def scenario(ros, node, feed):
        from dataclasses import replace
        node.drive_config = replace(node.drive_config, path_loss_recovery_enabled=False)
        node.path_search.config = node.drive_config
        feed(visible=True)
        assert feed(visible=True)['drive_reason'] == 'tracking'
        for _ in range(20):
            result = feed()
            assert result['drive_reason'] == 'path_execution_invalidated'
            assert not result['path_recovery']['active']
            assert json.loads(ros.published[SPORT][-1].parameter) == ZERO
    road_recovery(monkeypatch, scenario)


def test_road_legacy_smoother_also_uses_loss_recovery(monkeypatch):
    def scenario(ros, node, feed):
        assert feed(visible=True)['drive_reason'] == 'tracking'
        for _ in range(15):
            result = feed()
        assert result['drive_reason'] == 'path_recovery_scan_left'
        assert result['path_execution'] == {'enabled': False}
    road_recovery(monkeypatch, scenario, execution=False)


def test_search_cannot_resume_before_execution_confirmation_completes(monkeypatch):
    def scenario(ros, node, feed):
        for _ in range(4):
            result = feed(visible=True)
        assert result['drive_reason'] == 'tracking'
        for _ in range(15):
            feed()
        for count in (1, 2, 3):
            result = feed(visible=True)
            assert result['drive_reason'] == 'path_recovery_confirming'
            assert result['path_execution']['candidate_confirmations'] == count
            assert json.loads(ros.published[SPORT][-1].parameter) == ZERO
        assert feed(visible=True)['drive_reason'] == 'tracking'
    road_recovery(monkeypatch, scenario, confirm_frames=4)

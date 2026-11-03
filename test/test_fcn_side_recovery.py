"""Side-only recovery must rotate without fabricating a forward path."""
import json
from types import SimpleNamespace

import numpy as np
import pytest

from dangjin_fcn import FcnDrive
from fcn360_road import SIDE_RECOVERY_MAX_SECONDS, side_recovery_yaw
from test_fcn360_road import ARGS, SHA, lane, mask_topic, packet
from test_apriltag_task_stop_ros import RosHarness, SPORT, ZERO, debug
from test_surface_model_routing import start, cancel
from test_task_inference_lifecycle_ros import wait_for_count


def side_only(direction='left', shape=(16, 32)):
    masks = np.full((4, *shape), 11, np.uint8)
    scale = shape[0] // 8
    view = 1 if direction == 'left' else 3
    masks[view, 5*scale:8*scale, 6*scale:10*scale] = 3
    return masks


@pytest.mark.parametrize('shape', [(8,16), (16,32)])
@pytest.mark.parametrize('direction,sign', [('left',1), ('right',-1)])
def test_no_front_path_rotates_to_more_connected_road_after_fresh_confirmation(shape,direction,sign):
    drive = FcnDrive(1.6,10.)
    masks = side_only(direction,shape)
    assert drive.command(masks,1.,10.)[0].yaw_rate == 0.
    cmd,info,points = drive.command(masks,1.1,10.)
    assert (cmd.vx,cmd.vy,cmd.yaw_rate) == (0.,0.,sign*.18)
    assert cmd.reason == 'road_side_recovery' and not points
    assert not info['front_path_valid'] and not info['candidate_path_valid']
    assert info['recovery_direction'] == direction and info['recovery_status'] == 'rotating'


def test_ties_weak_islands_and_complete_dropout_never_authorize_a_turn():
    drive = FcnDrive(1.6,10.)
    tied = side_only()
    tied[3] = tied[1]
    weak = np.full((4,8,16),11,np.uint8)
    weak[1,5,6] = weak[1,6,8] = weak[1,7,6] = 3
    close = np.full((4,8,16),11,np.uint8)
    close[1,5:8,2:8] = close[3,5:8,2:8] = 3
    close[3,5,2] = 11
    for masks in (tied,close,weak,np.zeros((4,16,32),np.uint8)):
        drive.reset()
        for i in range(6):
            cmd,info,points = drive.command(masks,1.+i*.1,10.)
            assert (cmd.vx,cmd.vy,cmd.yaw_rate) == (0.,0.,0.) and not points
            assert info['recovery_status'] != 'rotating'


def test_total_side_road_extent_selects_the_larger_connected_region():
    drive = FcnDrive(1.6,10.)
    masks = side_only(shape=(8,16))
    masks[3,5:8,2:10] = 3
    drive.command(masks,1.,10.)
    cmd,info,_ = drive.command(masks,1.1,10.)
    assert info['recovery_right_road_fraction'] > info['recovery_left_road_fraction']
    assert cmd.yaw_rate < 0 and cmd.vx == cmd.vy == 0.


def test_direction_reversal_and_dropout_stop_before_reconfirming():
    drive = FcnDrive(1.6,10.)
    drive.command(side_only(),1.,10.)
    assert drive.command(side_only(),1.1,10.)[0].yaw_rate > 0
    assert drive.command(side_only('right'),1.2,10.)[0].yaw_rate == 0
    cmd,info,_ = drive.command(side_only('right'),1.3,10.)
    assert cmd.yaw_rate < 0
    assert side_recovery_yaw(info,.1) == -.1
    assert side_recovery_yaw(info,age_sec=info['recovery_remaining_sec']) == 0
    assert drive.command(np.zeros((4,16,32),np.uint8),1.4,10.)[0].yaw_rate == 0
    assert drive.command(side_only('right'),1.5,10.)[0].yaw_rate == 0


def test_duplicate_frames_cannot_start_recovery_and_valid_path_ends_it():
    drive = FcnDrive(1.6,10.)
    for _ in range(5):
        assert drive.command(side_only(),1.,10.)[0].yaw_rate == 0
    assert drive.command(side_only(),1.1,10.)[0].yaw_rate > 0
    cmd,info,points = drive.command(np.full((4,16,32),3,np.uint8),1.2,10.)
    assert cmd.vx > 0 and points and info['recovery_status'] == 'inactive'
    assert drive.command(side_only(),1.3,10.)[0].yaw_rate == 0


def test_recovery_time_budget_cannot_be_extended_by_short_visibility_losses():
    drive = FcnDrive(1.6,10.)
    for i in range(111):
        masks = side_only() if i%10 else np.zeros((4,16,32),np.uint8)
        cmd,info,_ = drive.command(masks,1.+i*.1,10.)
    assert (cmd.vx,cmd.yaw_rate) == (0.,0.) and info['recovery_status'] == 'timeout'
    assert info['recovery_elapsed_sec'] >= SIDE_RECOVERY_MAX_SECONDS


def test_ros_recovery_obeys_staleness_dropouts_regained_path_and_cancellation(monkeypatch):
    ros = RosHarness(monkeypatch)
    for check in debug.AUTOMATIC_STOP_CHECKS:
        monkeypatch.setitem(debug.ENV,'LINE_TRACKING_STOP_ON_'+check.upper(),'false')

    def scenario(node):
        node.destroy_subscription = lambda sub: ros.subscriptions.pop(mask_topic(SHA),None)
        start(ros,1,'recover-road')
        callback = ros.subscriptions[mask_topic(SHA)]
        count = 0
        def publish(grids):
            nonlocal count
            ros.now += .1
            callback(packet(ros,grids));count += 1;wait_for_count(ros,count)
            return json.loads(ros.published[SPORT][-1].parameter)
        assert publish(side_only(shape=(8,16))) == ZERO
        command = publish(side_only(shape=(8,16)))
        assert command == {'x':0.,'y':0.,'z':.18}
        assert ros.metrics()['drive_reason'] == 'fcn_side_recovery'
        assert not ros.metrics()['fcn360']['front_path_valid']
        ros.now += 1.2;node.publish_state()
        assert json.loads(ros.published[SPORT][-1].parameter) == ZERO
        assert ros.metrics()['drive_reason'] == 'fcn_masks_stale'
        publish(side_only(shape=(8,16)))
        assert publish(side_only(shape=(8,16)))['z'] > 0
        assert publish(np.zeros((4,8,16),np.uint8)) == ZERO
        assert publish(lane())['x'] > 0
        assert ros.metrics()['fcn360']['recovery_status'] == 'inactive'
        publish(side_only(shape=(8,16)));publish(side_only(shape=(8,16)))
        cancel(ros,'recover-road')
        callback(packet(ros,side_only(shape=(8,16))))
        assert json.loads(ros.published[SPORT][-1].parameter) == ZERO
    ros.run(scenario,*ARGS)


def test_simulation_packet_has_rotation_without_a_path_then_resumes_tracking():
    from dangjin_sim_server import SimulationService, encode_png
    from test_dangjin_sim import ROOT, request
    site = json.loads((ROOT/'environments/dangjin/site.json').read_text())
    world = json.loads((ROOT/'environments/dangjin/world.json').read_text())
    classes = side_only()
    detector = SimpleNamespace(details=dict(backend='fcn1024-360',input_width=1024,
        weights_sha256=SHA,views=4),predict_classes=lambda _: (classes.copy(),0.))
    sim = SimulationService(None,site,detector,world,FcnDrive(1.6,10.))
    req = request();del req['image_png']
    req.update(speed_mps=10.,views_atlas_png=encode_png(np.zeros((1024,2048,3),np.uint8)))
    assert sim.step(req)['command']['yaw_rate'] == 0
    req['stamp_sec'] += .1
    reply = sim.step(req)
    assert reply['command'] == dict(vx=0.,vy=0.,yaw_rate=.18,reason='road_side_recovery')
    assert not reply['paths'] and not reply['policy_metrics']['front_path_valid']
    classes[:] = 3;req['stamp_sec'] += .1
    resumed = sim.step(req)
    assert resumed['command']['vx'] > 0 and resumed['paths']['1']['points_xy']
    classes[:] = 0;req['stamp_sec'] += .1
    stopped = sim.step(req)['command']
    assert stopped['vx'] == stopped['vy'] == stopped['yaw_rate'] == 0.

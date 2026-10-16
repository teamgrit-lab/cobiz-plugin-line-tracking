"""Current grass/curb evidence bounds the road and smoothed Path."""
from dataclasses import replace
from pathlib import Path
import sys
import numpy as np
import pytest
import torch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
import local_path as lp
import swin_l_local_path_debug as debug
from test_accelerator_postprocess import device,make_segmenter


@pytest.fixture
def metric_grid(monkeypatch):
    xs=np.linspace(3.,8.,151,dtype=np.float32);ys=np.linspace(5.,-5.,201,dtype=np.float32)
    mx,my=np.meshgrid(np.arange(len(ys),dtype=np.float32),np.arange(len(xs),dtype=np.float32))
    def geometry(_shape,cfg):
        return mx,my,np.ones(mx.shape,np.uint8),xs,ys,replace(cfg,
            ground_half_width_m=5.,search_half_width_m=5.,bev_width_px=len(ys),bev_height_px=len(xs))
    monkeypatch.setattr(lp,'_road_birdseye_geometry',geometry)
    road=np.broadcast_to(abs(ys)<=2.,mx.shape).astype(np.uint8).copy()
    return xs,ys,road,lp.LocalPathConfig(close_kernel_px=0,branch_preference='center')


def test_one_forward_pass_supplies_fresh_curb_and_vegetation_without_changing_surface(device):
    model=make_segmenter(device);model.road_hazard_ids=(0,5);model.temporal_alpha=.1
    reference=make_segmenter(device);reference.temporal_alpha=.1
    scores=torch.zeros((6,32,48),device=device);scores[1]=10
    calls=[]
    def semantic(*_args,**_kwargs):calls.append(1);return scores
    model._semantic_scores=semantic;reference._semantic_scores=lambda *_args,**_kwargs:scores
    frame=np.zeros((32,48,3),np.uint8)
    first=model.segment(frame,detect_road_hazards=True)
    assert not first.road_hazard_mask.any()
    np.testing.assert_array_equal(first.selected_mask,reference.segment(frame).selected_mask)
    scores=scores.clone();scores[0,5:15,5:15]=20;scores[5,20:30,25:35]=20
    second=model.segment(frame,detect_road_hazards=True)
    assert len(calls)==2 and second.road_hazard_mask.sum()==200
    # The slow surface EMA still says road where the current hazard wins.
    assert np.all(second.selected_mask[5:15,5:15]==1)
    np.testing.assert_array_equal(second.selected_mask,reference.segment(frame).selected_mask)
    scores[0]=0;scores[5]=0
    assert not model.segment(frame,detect_road_hazards=True).road_hazard_mask.any()
    assert model.segment(frame).road_hazard_mask is None


def test_hazard_threshold_and_missing_labels_are_explicit():
    model=make_segmenter(torch.device('cpu'));model.road_hazard_ids=(5,)
    scores=torch.ones((6,32,48));scores[5]=1.1
    model._semantic_scores=lambda *_args,**_kwargs:scores;frame=np.zeros((32,48,3),np.uint8)
    assert not model.segment(frame,detect_road_hazards=True).road_hazard_mask.any()
    del model.road_hazard_ids
    assert not model.segment(frame,detect_road_hazards=True).road_hazard_mask.any()
    for value in (0.,1.01,float('nan')):
        with pytest.raises(ValueError):model.segment(frame,detect_road_hazards=True,road_hazard_min_confidence=value)


def test_boundary_margin_is_reserved_and_narrow_road_is_rejected(metric_grid):
    xs,ys,road,cfg=metric_grid
    result=lp.extract_road_centerline(road,cfg)
    assert result is not None and abs(result.points_xy[:,1]).max()<.1
    assert result.road_corridor[:,1].min()>=-1.45
    assert result.road_corridor[:,2].max()<=1.45
    assert lp.road_path_within_corridor(result.points_xy,result.road_corridor)
    narrow=np.broadcast_to(abs(ys)<.5,road.shape).astype(np.uint8)
    assert lp.extract_road_centerline(narrow,cfg) is None
    assert lp.extract_road_centerline(narrow,replace(cfg,road_clearance_enabled=False)) is not None


@pytest.mark.parametrize('sign',[-1,1])
def test_current_hazard_overrides_a_road_ghost_and_moves_reference_away(metric_grid,sign):
    xs,ys,road,cfg=metric_grid
    hazard=np.broadcast_to((sign*ys>=1.2)&(sign*ys<=2.5),road.shape).copy()
    result=lp.extract_road_centerline(road,cfg,hazard)
    assert result is not None and np.median(result.points_xy[:,1])*sign<-.25
    assert np.max(sign*result.points_xy[:,1])<=.65
    assert result.road_clearance['hazard_pixels']>0
    assert np.array_equal(road,np.broadcast_to(abs(ys)<=2.,road.shape))


def test_full_width_curb_cannot_link_to_the_road_behind_it(metric_grid):
    xs,ys,road,cfg=metric_grid
    curb=np.broadcast_to(abs(xs[:,None]-5.5)<.04,road.shape).copy()
    result=lp.extract_road_centerline(road,cfg,curb)
    assert result is not None and result.points_xy[-1,0]<4.9
    close_curb=np.broadcast_to(abs(xs[:,None]-4.5)<.04,road.shape).copy()
    assert lp.extract_road_centerline(road,cfg,close_curb) is None


def test_smoothing_cannot_retain_a_path_inside_new_grass(metric_grid):
    xs,ys,road,cfg=metric_grid;smoother=lp.LocalPathSmoother(cfg)
    baseline=lp.extract_road_centerline(road,cfg);smoother.update(baseline,0.)
    hazard=np.broadcast_to(ys>=0.,road.shape).copy()
    estimate=lp.extract_road_centerline(road,cfg,hazard)
    updated=smoother.update(estimate,.1)
    assert updated is not None and updated.road_clearance['history_replanned']
    assert not updated.road_clearance['path_clamped']
    assert lp.road_path_within_corridor(updated.points_xy,estimate.road_corridor)
    assert updated.points_xy[:,1].max()<-.55
    assert lp.road_path_within_corridor(smoother.current(.3).points_xy,estimate.road_corridor)


def test_dense_corridor_detects_a_hazard_between_path_endpoints():
    points=np.array([[3.,0.],[8.,0.]])
    corridor=np.array([[3.,-1.,1.],[5.,.5,1.],[8.,-1.,1.]])
    assert not lp.road_path_within_corridor(points,corridor)


def test_nonroad_modes_do_not_receive_the_hazard_mask(metric_grid):
    xs,ys,road,cfg=metric_grid
    selected=road.copy();hazard=np.ones_like(road)
    # Sidewalk/union geometry uses the normal image projection, not this mock BEV.
    selected=np.ones(road.shape,np.uint8);hazard=np.ones_like(selected)
    before=debug.extract_path_estimates(selected,(0,1,2),cfg)
    after=debug.extract_path_estimates(selected,(0,1,2),cfg,road_hazard_mask=hazard)
    assert after[1] is None
    np.testing.assert_array_equal(after[0].points_xy,before[0].points_xy)
    assert before[2] is after[2] is None


def test_fresh_unsafe_road_clears_a_restricted_path_hold(metric_grid):
    xs,ys,road,cfg=metric_grid;smoother=lp.LocalPathSmoother(cfg)
    smoother.update(lp.extract_road_centerline(road,cfg),0.)
    assert smoother.current(.05) is not None
    assert smoother.update(None,.1) is None
    assert smoother.current(.2) is None


@pytest.mark.parametrize('key,value',[('EDGE_MARGIN_M','nan'),('HAZARD_MARGIN_M','-1'),
    ('HAZARD_MIN_CONFIDENCE','0'),('HAZARD_MIN_CONFIDENCE','1.1')])
def test_invalid_clearance_env_rejected(monkeypatch,key,value):
    monkeypatch.setitem(debug.ENV,'SWIN_L_ROAD_'+key,value)
    with pytest.raises(SystemExit):debug.parse_args(['task-drive'])


def test_clearance_env_and_rectangle_defaults(monkeypatch):
    monkeypatch.setitem(debug.ENV,'SWIN_L_ROAD_EDGE_MARGIN_M','0.45')
    monkeypatch.setitem(debug.ENV,'SWIN_L_ROAD_HAZARD_MARGIN_M','0.8')
    cfg=debug._local_path_config_from_args(debug.parse_args(['task-drive']))
    assert cfg.road_edge_margin_m==.45 and cfg.road_hazard_margin_m==.8
    assert cfg.road_roi_polygon==lp.DEFAULT_ROAD_ROI_POLYGON


@pytest.mark.parametrize('enabled',[True,False])
def test_clearance_block_stops_immediately_without_semantic_forward_grace(enabled):
    from swin_l_drive_control import DriveConfig,decide_drive
    cfg=replace(DriveConfig(),path_loss_recovery_enabled=enabled)
    out=decide_drive(None,camera_age_sec=0.,inference_age_sec=0.,config=cfg,
        last_valid_forward_mps=.8,last_valid_yaw_rate=.1,path_recovery_inferences=1,
        road_clearance_blocked=True)
    assert (out.vx,out.vy,out.yaw_rate)==(0.,0.,0.)
    assert out.reason=='road_clearance_blocked'


@pytest.mark.parametrize('selected',[0,1,2])
def test_runtime_requests_hazards_only_for_road_and_blocks_forward(monkeypatch,selected):
    import json
    from types import SimpleNamespace as NS
    from test_lidar_obstacles_ros import enabled_ros,complete_inference
    from test_apriltag_task_stop_ros import SPORT
    from test_road_center_tracking import projected_road
    ros=enabled_ros(monkeypatch)
    monkeypatch.setitem(debug.ENV,'LINE_TRACKING_AVOIDANCE_ENABLED','false')
    monkeypatch.setitem(debug.ENV,'LINE_TRACKING_PATH_LOSS_RECOVERY_ENABLED','true')
    cfg=lp.LocalPathConfig();region=projected_road(cfg,2.,-2.)
    # Sidewalk modes get the same shape with their own selected surface.
    labels=region*(2 if selected==2 else 1)
    hazard=np.zeros_like(region,bool);calls=[]
    def segment(_frame,**kwargs):
        calls.append(kwargs)
        return NS(selected_mask=labels.copy(),road_hazard_mask=hazard.copy(),
                  inference_seconds=.01)
    monkeypatch.setattr(debug,'BestSoFarSegmenter',lambda _:NS(
        device=NS(type='cuda'),reset=lambda:None,segment=segment))
    def scenario(node):
        ros.start();ros.now=2.1
        first=complete_inference(node,ros)
        assert calls[-1].get('detect_road_hazards',False)==(selected == 1)
        assert first['drive_reason'] in ('tracking','tracking_slow_turn')
        assert json.loads(ros.published[SPORT][-1].parameter)['x']>0
        # Current grass/curb evidence conflicts with retained road labels.
        hazard[:]=True;ros.now+=.1
        after=complete_inference(node,ros)
        if selected == 1:
            move=json.loads(ros.published[SPORT][-1].parameter)
            assert move=={'x':0.,'y':0.,'z':0.}
            assert after['road_clearance']['blocked']
            assert after['drive_reason']=='road_clearance_blocked'
            # A confirmed exclusion must not turn into automatic rotation.
            ros.now+=2.;node.publish_state()
            assert ros.metrics()['drive_reason']=='road_clearance_blocked'
            assert not ros.metrics()['path_recovery']['active']
        else:
            assert not after['road_clearance']['enabled']
            assert after['drive_reason'] in ('tracking','tracking_slow_turn')
        hazard[:]=False;ros.now+=.1
        # A newly accepted task must not inherit the prior blocked state.
        node.on_task_event(NS(data=json.dumps({'type':'TASK_ABORTED','task_id':'tag-stop-1'})))
        ros.now+=1.1
        node.publish_state()
        ros.start(task_id='clear-road-2')
        node.publish_state()
        assert not ros.metrics()['road_clearance']['blocked']
        complete_inference(node,ros)
        assert not ros.metrics()['road_clearance']['blocked']
    ros.run(scenario,'--path-mask-class',str(selected),'--branch-preference','center')

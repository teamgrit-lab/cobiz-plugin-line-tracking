"""Native 1024 scores, transport, occlusion and control must keep their resolution."""
import hashlib
import json
import sys
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from dangjin_fcn import FcnPerception, FcnDrive, apply_class_faults
from dangjin_sensor_faults import SensorFaults, fault_config
from dangjin_sim_server import decode_surround, encode_png, SimulationService
from fcn360_road import (BLIND, EGO, FRAME_ID, FcnRoadPlanner, ROAD, apply_front_robot_prior,
                         camera_maps, classify_panorama, front_robot_mask, model_shapes,
                         observed_classes, read_masks)
from local_path import LocalPathConfig
from surround_camera import load_profile, profile_sha256
from test_dangjin_sim import ROOT, request
from test_fcn360_road import lane, packet, SHA
from test_apriltag_task_stop_ros import RosHarness, SPORT, ZERO, debug
from test_surface_model_routing import start, cancel
from test_task_inference_lifecycle_ros import image, wait_for_count


def native_message(classes):
    h, w = classes.shape[1:]
    return SimpleNamespace(header=SimpleNamespace(frame_id=FRAME_ID),encoding='mono8',
                           height=4*h,width=w,step=w,data=classes.tobytes())


def test_native_packet_and_projection_equal_legacy_geometry_without_pooling():
    raw = lane()
    raw[0,4,11] = 14
    native = raw.repeat(2,axis=1).repeat(2,axis=2)
    decoded = read_masks(native_message(native),1024)
    assert decoded.shape == (4,16,32)
    assert (decoded[0] == EGO).sum() == 24
    assert np.all(observed_classes(decoded)[0,front_robot_mask(16,32)] == BLIND)
    for grids in (raw,native):
        planner = FcnRoadPlanner(LocalPathConfig(),1.6)
        path,metrics = planner.plan(apply_front_robot_prior(grids),stamp=1.)
        assert path is not None
        if grids is raw: expected_path,expected_metrics = path.points_xy,metrics
        else:
            np.testing.assert_allclose(path.points_xy,expected_path,atol=1e-6)
            assert metrics == expected_metrics
    # Two different native cells in one legacy block remain distinct.
    native[0,4,2],native[0,4,3] = ROAD,11
    out = read_masks(native_message(native),1024)
    assert (out[0,4,2],out[0,4,3]) == (ROAD,11)
    with pytest.raises(ValueError,match='resolution'):
        read_masks(native_message(native),512)


@pytest.mark.parametrize('width',[512,1024])
def test_real_projector_and_browser_use_native_scores_and_same_ego_region(width):
    shape,output = model_shapes(width)
    def infer(tensor):
        assert tensor.shape == shape and tensor.dtype == np.float32
        scores = np.zeros(output,np.float32);scores[:,3] = 1
        return scores
    panorama = np.full((1440,2880,3),[17,123,246],np.uint8)
    jetson,views = classify_panorama(panorama,infer,width)
    assert all(v.shape == (width//2,width,3) and np.all(v == [17,123,246]) for v in views)
    browser = FcnPerception.__new__(FcnPerception)
    browser.input_width,browser.input_height,browser.logits = width,width//2,infer
    actual,_ = browser.predict_classes(views)
    np.testing.assert_array_equal(actual,jetson)
    assert actual.shape == (4,width//64,width//32)
    assert np.count_nonzero(actual == EGO) == (6 if width == 512 else 24)
    assert len(camera_maps(width)) == 4


def test_1024_surround_atlas_is_exact_bounded_and_keeps_legacy_decoder_limit():
    from connectome_server import decode_image
    panels = [np.full((512,1024,3),k,np.uint8) for k in (11,22,33,44)]
    atlas = np.vstack([np.hstack(panels[:2]),np.hstack(panels[2:])])
    req = dict(views_atlas_png=encode_png(atlas),camera_profile_sha256=profile_sha256(load_profile()))
    views = decode_surround(req,load_profile(),width=1024,height=512)
    for actual,expected in zip(views,panels):np.testing.assert_array_equal(actual,expected)
    with pytest.raises(ValueError,match='bounded PNG'):
        decode_image(req['views_atlas_png'],cv2.IMREAD_COLOR)
    req['views_atlas_png'] = encode_png(np.zeros((1025,2048,3),np.uint8))
    with pytest.raises(ValueError):decode_surround(req,load_profile(),width=1024,height=512)


def test_native_faults_and_ros_preview_do_not_resize_controller_grids():
    from fcn360_detector import overlay
    classes = apply_front_robot_prior(np.full((4,16,32),3,np.uint8))
    faults = SensorFaults(fault_config(dict(enabled=True,profile='normal',seed=47)))
    saw_dropout = False
    for elapsed in np.arange(0,35,.5):
        after,info = apply_class_faults(classes,faults,float(elapsed))
        assert after.shape == classes.shape
        np.testing.assert_array_equal(after == EGO,classes == EGO)
        if info['kind'] == 'dropout':
            saw_dropout = True
            assert not (after == ROAD).any()
            assert FcnDrive(1.6,2.).command(after,1.,2.)[0].vx == 0
    assert saw_dropout
    preview = overlay([np.zeros((512,1024,3),np.uint8)]*4,classes)
    assert preview.shape == (512,1024,3)
    np.testing.assert_array_equal(preview[180,272],(100,35,115))


def test_model_package_binds_configuration_hash_and_resolution(tmp_path,monkeypatch):
    model=b'onnx';config=json.dumps({'dataset_contract':{'shape':[512,1024]}}).encode()
    (tmp_path/'fcn_resnet18.onnx').write_bytes(model);(tmp_path/'config.json').write_bytes(config)
    manifest={'sha256':{'fcn_resnet18.onnx':hashlib.sha256(model).hexdigest(),
                        'config.json':hashlib.sha256(config).hexdigest()}}
    (tmp_path/'complete.json').write_text(json.dumps(manifest))
    monkeypatch.setattr(cv2.dnn,'readNetFromONNX',lambda _:object())
    perception=FcnPerception(tmp_path)
    assert perception.details['backend']=='fcn1024-360'
    assert perception.details['class_grid_shape']==[4,16,32]
    assert perception.details['fixed_front_robot_columns']==[14,18]
    assert perception.details['fixed_front_robot_rows']==[10,16]
    assert perception.input_width==1024
    (tmp_path/'config.json').write_text('{"dataset_contract":{"shape":[256,512]}}')
    with pytest.raises(ValueError,match='config.*manifest'):FcnPerception(tmp_path)


def test_1024_road_task_routes_native_packets_and_sidewalk_still_uses_legacy(monkeypatch):
    ros=RosHarness(monkeypatch)
    for check in debug.AUTOMATIC_STOP_CHECKS:
        monkeypatch.setitem(debug.ENV,'LINE_TRACKING_STOP_ON_'+check.upper(),'false')
    side_calls=[]
    def segment(frame,path_mask_class,**kwargs):
        side_calls.append(path_mask_class)
        return SimpleNamespace(selected_mask=np.full((360,640),2,np.uint8),inference_seconds=.01)
    monkeypatch.setattr(debug,'SegmentationModel',lambda _:SimpleNamespace(
        device=SimpleNamespace(type='cuda'),reset=lambda:None,segment=segment))
    def scenario(node):
        node.destroy_subscription=lambda sub:ros.subscriptions.pop('/line_tracking/fcn360/masks/model_'+SHA)
        start(ros,1,'native-road');callback=ros.subscriptions['/line_tracking/fcn360/masks/model_'+SHA]
        grids=lane().repeat(2,axis=1).repeat(2,axis=2)
        msg=packet(ros,grids);msg.height,msg.width,msg.step=64,32,32
        callback(msg);wait_for_count(ros,1)
        command=json.loads(ros.published[SPORT][-1].parameter)
        assert command['x']>0 and command['y']==0
        assert ros.metrics()['inference_backend']=='fcn1024-360'
        ros.now+=1.2;node.publish_state()
        assert json.loads(ros.published[SPORT][-1].parameter)==ZERO
        cancel(ros,'native-road');start(ros,2,'legacy-sidewalk');image(ros);wait_for_count(ros,2)
        assert side_calls==[2] and ros.metrics()['inference_backend']=='legacy'
    ros.run(scenario,'--fcn-sha256',SHA,'--path-mask-class','1','--path-planner','local')


def test_tensorrt_uses_native_bindings_and_separate_resolution_caches(tmp_path,monkeypatch):
    """Validate allocation/cache contracts with fake GPU bindings, not GPU performance."""
    from fcn360_detector import TrtFcn
    state={'width':1024,'builds':0}
    class Tensor:
        def __init__(self,shape):self.shape=shape
        def data_ptr(self):return id(self)
    class Engine:
        num_io_tensors=2
        def get_tensor_shape(self,name):return model_shapes(state['width'])[name=='output_0']
        def get_tensor_dtype(self,name):return 'float32'
        def create_execution_context(self):return SimpleNamespace(set_tensor_address=lambda *a:True)
    class Runtime:
        def __init__(self,*a):pass
        def deserialize_cuda_engine(self,raw):return Engine()
    class Logger:
        WARNING=1
        def __init__(self,*a):pass
    class Parser:
        def __init__(self,*a):pass
        def parse(self,*a):return True
    class Builder:
        def __init__(self,*a):pass
        def create_network(self,*a):return object()
        def create_builder_config(self):
            return SimpleNamespace(set_memory_pool_limit=lambda *a:None,clear_flag=lambda *a:None)
        def build_serialized_network(self,*a):state['builds']+=1;return b'fake engine'
    trt=SimpleNamespace(__version__='10.test',Logger=Logger,Runtime=Runtime,Builder=Builder,
        OnnxParser=Parser,float32='float32',MemoryPoolType=SimpleNamespace(WORKSPACE=0),
        BuilderFlag=SimpleNamespace(TF32=0))
    torch=SimpleNamespace(cuda=SimpleNamespace(get_device_properties=lambda _:SimpleNamespace(name='test GPU',major=8,minor=7)),
        version=SimpleNamespace(cuda='12.6'),float32='float32',empty=lambda shape,**kw:Tensor(shape))
    monkeypatch.setitem(sys.modules,'tensorrt',trt);monkeypatch.setitem(sys.modules,'torch',torch)
    model=tmp_path/'model.onnx';model.write_bytes(b'fake ONNX')
    digest=hashlib.sha256(model.read_bytes()).hexdigest();cache=tmp_path/'engines'
    native=TrtFcn(model,digest,cache)
    assert native.input.shape==(1,3,512,1024) and native.output.shape==(1,21,16,32)
    TrtFcn(model,digest,cache)
    assert state['builds']==1 and len(list(cache.glob('*.plan')))==1
    state['width']=512
    TrtFcn(model,digest,cache,512)
    assert state['builds']==2 and len(list(cache.glob('*.plan')))==2
    # An incompatible graph is rejected before creating a reusable cache entry.
    with pytest.raises(ValueError,match='contract'):TrtFcn(model,digest,tmp_path/'bad',1024)
    assert not list((tmp_path/'bad').glob('*.plan'))

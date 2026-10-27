"""Meaningful invariants for raw timestamps, geometry and label semantics."""
import sys
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from build_road_sample_dataset import raw_xyz, point_times
from label_road_sample_dataset import (
    ground_plane, unproject_ground, project_points, interpolate_annotations,
    propose_command,
)


@pytest.mark.parametrize('big_endian', [False, True])
def test_padded_cloud_preserves_zero_point_and_per_point_time_alignment(big_endian):
    endian = '>' if big_endian else '<'
    data = bytearray(2*60)  # two 26-byte points plus eight padding bytes per row
    dtype = np.dtype({'names':['x','y','z','timestamp'], 'formats':[endian+'f4']*3+[endian+'f8'],
                      'offsets':[0,4,8,18], 'itemsize':26})
    a = np.ndarray((2,2), dtype=dtype, buffer=data, strides=(60,26))
    a['x'] = [[0,1],[2,3]];a['y'] = [[0,4],[5,6]];a['z'] = [[0,7],[8,9]]
    times = 1_789_454_000_000_000_000 + np.arange(4)*25_000_000
    a['timestamp'] = times.reshape(2,2)
    fields=[NS(name=n, offset=o, datatype=t) for n,o,t in [('x',0,7),('y',4,7),('z',8,7),('timestamp',18,8)]]
    m=NS(width=2,height=2,point_step=26,row_step=60,is_bigendian=big_endian,data=data,fields=fields,
         header=NS(stamp=NS(sec=1_789_454_000,nanosec=0)))
    xyz=raw_xyz(m);t=point_times(m)
    assert xyz.shape==(4,3) and t.shape==(4,)
    np.testing.assert_array_equal(xyz,[[0,0,0],[1,4,7],[2,5,8],[3,6,9]])
    np.testing.assert_array_equal(t,times.astype(float))
    a['timestamp']/=1e9
    with pytest.raises(ValueError,match='units or header mismatch'):point_times(m)


def test_ground_plane_rejects_wall_and_recovers_sloped_noisy_road():
    rng=np.random.default_rng(42)
    xy=np.column_stack([rng.uniform(.8,8,3000),rng.uniform(-2.8,2.8,3000)])
    road=np.column_stack([xy,.025*xy[:,0]-.012*xy[:,1]-.45+rng.normal(0,.008,len(xy))])
    wall=np.column_stack([rng.uniform(.8,8,500),rng.uniform(-2.8,2.8,500),rng.uniform(-.2,1.5,500)])
    c,report=ground_plane(np.concatenate([road,wall]))
    assert report['valid']
    np.testing.assert_allclose(c,[.025,-.012,-.45],atol=.006)
    assert ground_plane(np.zeros((10,3)))[0] is None


def test_projection_ground_intersection_roundtrip_and_right_turn_sign():
    K=np.array([[535.,0,643.],[0,533.,356.],[0,0,1.]])
    camera=np.array([[0,-1,0,.03],[0,0,-1,.05],[1,0,0,-.34],[0,0,0,1.]])
    road=np.array([.02,.01,-.45]);points=np.array([[4.,-.6,0.],[4.,.6,0.]])
    points[:,2]=points[:,:2]@road[:2]+road[2]
    uv,visible=project_points(points,K,np.zeros(8),camera)
    recovered=unproject_ground(uv,K,np.zeros(8),camera,road)
    assert visible.all()
    np.testing.assert_allclose(recovered,points,atol=1e-7)
    ev={'valid':True,'ground_support_fraction':1.,'nearest_supported_obstacle_along_path_m':None}
    right=propose_command('right_turn',points[0],ev,True,None)
    left=propose_command('straight',points[1],ev,True,None)
    assert right['action']=='turn_right' and right['angular_z_radps']<0
    assert left['action']=='turn_left' and left['angular_z_radps']>0
    assert right['is_estimated'] and not right['human_verified'] and not right['deployable']


@pytest.mark.parametrize('ev,valid,annotation,reason',[
    ({'valid':True,'ground_support_fraction':1.,'nearest_supported_obstacle_along_path_m':None},True,{'reason':'vehicle_crossing'},'vehicle_crossing'),
    ({'valid':True,'ground_support_fraction':.3,'nearest_supported_obstacle_along_path_m':None},True,None,'insufficient_ground_support'),
    ({'valid':True,'ground_support_fraction':1.,'nearest_supported_obstacle_along_path_m':1.},True,None,'near_non_ground_returns_in_proposed_corridor'),
    ({'valid':False},True,None,'insufficient_geometry'),
    ({'valid':True},False,None,'unreliable_camera_lidar_association'),
])
def test_stop_conditions_do_not_turn_absence_of_evidence_into_forward_command(ev,valid,annotation,reason):
    result=propose_command('straight',[4.,0.,-.4],ev,valid,annotation)
    assert result['stop'] and result['linear_x_mps']==result['angular_z_radps']==0
    assert result['stop_reason']==reason


def test_annotation_interpolation_uses_acquisition_time_and_marks_offline_context():
    rows=[{'camera':{'index':i,'header_time_ns':str(t)}} for i,t in enumerate([1000,1250,2000])]
    keyframes=[{'index':0,'pixels':[[0,10]]},{'index':2,'pixels':[[100,10]]}]
    result,p=interpolate_annotations(keyframes,rows[1],rows,'pixels')
    assert result==[[25.,10.]]
    assert p['offline_context_used'] and not p['human_verified']

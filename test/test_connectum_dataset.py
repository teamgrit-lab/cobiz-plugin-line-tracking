import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from PIL import Image

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from build_connectum_dataset import source_ids,nearest,write_image,raw_point_times_with_issue
from label_connectum_dataset import surface_target,make_command
from finalize_connectum_dataset import temporal_clips,SPLITS


def test_duplicate_filenames_share_recording_identity():
    a=source_ids({'path':'/data/a2_record_default_20260915_153503_10.mcap'})
    b=source_ids({'path':'/data/a2_record_default_20260915_153503_10-028.mcap'})
    assert a==b==('20260915_153503','20260915_153503_p010')


def test_nearest_scan_uses_acquisition_timestamps_and_handles_boundaries():
    assert nearest(['a','b','c'],[100,200,300],190)=='b'
    assert nearest(['a','b'],[100,200],0)=='a'
    assert nearest([],[],200) is None


def test_native_rgb_and_bgr_png_roundtrip(tmp_path):
    rgb=np.array([[[255,0,10],[20,250,30]],[[40,50,245],[255,255,255]]],np.uint8)
    for fmt,data in [('rgb24',rgb),('bgr24',rgb[:,:,::-1])]:
        path=tmp_path/f'{fmt}.png'
        write_image(path,SimpleNamespace(width=2,height=2,input_format=fmt,payload=data.tobytes()))
        np.testing.assert_array_equal(np.asarray(Image.open(path)),rgb)


def test_mixed_time_lidar_scan_is_preserved_and_quarantined():
    start=1_792_570_568_000_000_000
    times=np.array([start,start+50_000_000,start+1_900_000_000],dtype='<f8')
    message=SimpleNamespace(fields=[SimpleNamespace(name='timestamp',datatype=8,count=1,offset=0)],
        is_bigendian=False,point_step=8,row_step=24,height=1,width=3,data=times.tobytes(),
        header=SimpleNamespace(stamp=SimpleNamespace(sec=start//1_000_000_000,nanosec=start%1_000_000_000)))
    actual,issue=raw_point_times_with_issue(message)
    np.testing.assert_array_equal(actual,times)
    assert issue=='mixed_time_scan_over_200ms'
    message.data=np.array([start,start+50_000_000,start+99_000_000],dtype='<f8').tobytes()
    assert raw_point_times_with_issue(message)[1] is None


def test_visual_target_abstains_on_gap_and_separate_surfaces():
    mask=np.zeros((360,640),np.uint8);mask[180:,220:420]=1
    target,points,status=surface_target(mask)
    assert target==[319.5,216.] and status['status']=='candidate'
    mask[216]=255
    assert surface_target(mask)[0] is None
    assert surface_target(np.full((360,640),255,np.uint8))[0] is None


def test_missing_depth_or_support_is_abstention_not_false_stop_label():
    ev={'valid':True,'ground_support_fraction':1.,'nearest_supported_obstacle_along_path_m':None}
    assert make_command(None,{'valid':False},True,False)[0] is None
    assert make_command([3,0,-.4],ev,False,False)[0] is None
    weak={**ev,'ground_support_fraction':.2}
    assert make_command([3,0,-.4],weak,True,False)[0] is None
    left=make_command([3,1,-.4],ev,True,False)[0]
    assert left['angular_z_radps']>0 and left['action']=='turn_left'
    stop=make_command([3,0,-.4],ev,True,True)[0]
    assert stop['stop'] and stop['linear_x_mps']==stop['angular_z_radps']==0


def test_temporal_clips_never_cross_file_or_time_gap():
    def row(i,t,seq):return {'id':str(i),'camera':{'header_time_ns':str(round(t*1e9))},'sequence':seq,'session':'20260915_153503'}
    rows=[row(0,1,'a'),row(1,1.2,'a'),row(2,1.4,'a'),row(3,3,'a'),row(4,3.2,'a'),row(5,3.4,'b')]
    labels={str(i):{'estimated_command':None} for i in range(6)}
    clips=temporal_clips(rows,labels,length=3)
    assert len(clips)==1 and clips[0]['frame_ids']==['0','1','2']
    np.testing.assert_allclose(clips[0]['dt_s'],[0,.2,.2])
    assert set(SPLITS.values())=={'train','val','test','review'}

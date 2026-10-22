"""Real spawn/pipe transport: task reset, source identity and child failures."""
import sys
from pathlib import Path
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from perception_process import PerceptionProcess


def echo_worker(pipe, _config, _planner):
    pipe.send({'ready':True,'device':'cpu'})
    while True:
        r=pipe.recv()
        if r is None:break
        if r['sequence']==99:
            pipe.send({'error':'deliberate inference failure'});break
        pipe.send({'generation':r['generation'],'sequence':r['sequence'],
                   'result':(int(r['rgb'].sum()),r['reset'],r['stamp']), 'mode':r['mode'], 'memory':{}})
    pipe.close()


def bad_identity_worker(pipe,_config,_planner):
    pipe.send({'ready':True,'device':'cpu'});r=pipe.recv()
    pipe.send({'generation':r['generation']-1,'sequence':r['sequence'],'memory':{}})
    pipe.close()


def bad_mode_worker(pipe, _config, _planner):
    pipe.send({'ready': True, 'device': 'cpu'}); r = pipe.recv()
    pipe.send({'generation': r['generation'], 'sequence': r['sequence'], 'mode': 1, 'memory': {}})
    pipe.close()


def test_spawn_keeps_frame_and_generation_and_resets_temporal_state():
    worker=PerceptionProcess(None,{},target=echo_worker,startup_timeout=10)
    try:
        image=np.ones((3,4,3),np.uint8)
        assert worker.infer(image,2.,1,1)['result']==(36,True,2.)
        assert worker.infer(image,2.1,1,4,mode=1)['mode']==1
        assert worker.infer(image,2.5,1,2)['result']==(36,False,2.5)
        worker.reset()
        assert worker.infer(image,3.,2,3)['result']==(36,True,3.)
        with pytest.raises(RuntimeError,match='deliberate inference failure'):
            worker.infer(image,3.5,2,99)
    finally:worker.close()
    assert not worker.process.is_alive()


def test_spawn_rejects_a_reply_from_another_task():
    worker=PerceptionProcess(None,{},target=bad_identity_worker,startup_timeout=10)
    try:
        with pytest.raises(RuntimeError,match='does not match task/frame'):
            worker.infer(np.zeros((2,2,3),np.uint8),1.,2,1)
    finally:worker.close()


def test_spawn_rejects_a_road_reply_for_a_sidewalk_request():
    worker = PerceptionProcess(None, {}, target=bad_mode_worker, startup_timeout=10)
    try:
        with pytest.raises(RuntimeError, match='does not match task/frame'):
            worker.infer(np.zeros((2, 2, 3), np.uint8), 1., 2, 1, mode=2)
    finally:
        worker.close()

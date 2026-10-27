import sys
from pathlib import Path
import numpy as np
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from dangjin_sensor_faults import SensorFaults,fault_config


def config(seed=47,profile='normal',enabled=True):return dict(seed=seed,profile=profile,enabled=enabled)


def test_seed_reproduces_bursts_and_a_different_seed_changes_them():
    mask=np.ones((4,270,480),np.uint8);a=SensorFaults(config());b=SensorFaults(config());c=SensorFaults(config(48))
    kinds=set();different=False
    for t in np.arange(0,60,.5):
        x,dx=a.apply(mask,float(t));y,dy=b.apply(mask,float(t));z,dz=c.apply(mask,float(t))
        np.testing.assert_array_equal(x,y);assert dx==dy;kinds.add(dx['kind'])
        different|=not np.array_equal(x,z)
    assert different and kinds=={'clear','holes','dropout'}


def test_false_negatives_do_not_mutate_input_or_create_drivable_pixels():
    original=np.zeros((4,270,480),np.uint8);original[:,140:,80:400]=1;copy=original.copy()
    faults=SensorFaults(config());saw_holes=saw_dropout=False
    for t in np.arange(0,30,.5):
        out,info=faults.apply(original,float(t));assert np.all(out<=original)
        np.testing.assert_array_equal(original,copy)
        if info['kind']=='holes' and info['removed_fraction']>0:
            saw_holes=True;assert out.any() and 0<info['removed_fraction']<1
        if info['kind']=='dropout':
            saw_dropout=True;assert not out.any() and info['remaining_road_pixels']==[0]*4
    assert saw_holes and saw_dropout


def test_pausing_and_frame_rate_do_not_advance_or_redraw_the_pattern():
    mask=np.ones((4,270,480),np.uint8);dense=SensorFaults(config());sparse=SensorFaults(config())
    for t in np.arange(0,9,.1):dense.apply(mask,float(t))
    a,info=dense.apply(mask,9.);b,other=sparse.apply(mask,9.)
    np.testing.assert_array_equal(a,b);assert info==other
    again,repeated=dense.apply(mask,9.)
    np.testing.assert_array_equal(a,again);assert info==repeated


def test_effect_can_be_disabled_and_profiles_have_bounded_durations():
    mask=np.ones((4,270,480),np.uint8)
    for profile in ('mild','normal','severe'):
        faults=SensorFaults(config(profile=profile,enabled=False))
        out,info=faults.apply(mask,100.)
        np.testing.assert_array_equal(mask,out);assert info['kind']=='off' and info['removed_fraction']==0
        enabled=SensorFaults(config(profile=profile));enabled.apply(mask,120.)
        assert all(.5<=e['end']-e['start']<=3 for e in enabled.events['dropout'])


@pytest.mark.parametrize('bad',[{'enabled':1,'profile':'normal','seed':1},config(-1),config(2**32),config(1.5),config(True),config(profile='invalid')])
def test_invalid_fault_configuration_is_rejected(bad):
    with pytest.raises(ValueError):fault_config(bad)

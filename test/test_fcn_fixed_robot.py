"""Only the video-bounded front rig can be ego; it supplies no road evidence."""

import cv2
import numpy as np
import pytest

from fcn360_road import (BLIND, EGO, FRONT_BLIND, FRONT_ROBOT_ROWS, FcnRoadPlanner, apply_front_robot_prior,
                         classify_panorama, front_robot_mask, observed_classes)
from local_path import LocalPathConfig


@pytest.mark.parametrize('shape', [(8,16), (16,32)])
def test_fixed_front_columns_and_video_height_are_the_only_ego_region(shape):
    rng = np.random.default_rng(12)
    raw = rng.integers(0,21,size=(4,*shape),dtype=np.uint8)
    before = raw.copy()
    fixed = apply_front_robot_prior(raw)
    mask = front_robot_mask(*shape)
    expected = np.zeros(shape, bool)
    scale = shape[0] // 8
    expected[5*scale:8*scale,7*scale:9*scale] = True
    np.testing.assert_array_equal(mask,expected)
    assert mask.sum() == 6 * scale**2
    assert np.all(fixed[0,mask] == EGO)
    expected_ego = np.zeros_like(raw,dtype=bool)
    expected_ego[0] = mask
    np.testing.assert_array_equal(fixed == EGO,expected_ego)
    outside = ~expected_ego
    assert np.all(fixed[outside & (raw == EGO)] == 0)
    np.testing.assert_array_equal(fixed[outside & (raw != EGO)],raw[outside & (raw != EGO)])
    assert not (fixed[0,:FRONT_ROBOT_ROWS[0]] == EGO).any()
    np.testing.assert_array_equal(raw,before)
    np.testing.assert_array_equal(apply_front_robot_prior(fixed),fixed)


def test_fixed_robot_is_blind_for_planning_and_cannot_supply_its_own_pavement():
    raw = np.full((4,8,16),11,np.uint8)
    fixed = apply_front_robot_prior(raw)
    mask = front_robot_mask()
    observed = observed_classes(fixed)
    assert np.all(observed[0,mask] == BLIND)
    assert not (observed[0,mask] == 0).any()  # not an artificial detector-loss hole
    planner = FcnRoadPlanner(LocalPathConfig(),1.6)
    assert planner.plan(fixed,stamp=1.)[0] is None
    raw[:] = 3
    fixed = apply_front_robot_prior(raw)
    observed = observed_classes(fixed)
    np.testing.assert_array_equal(FRONT_BLIND,mask)
    assert np.all(observed[0,~mask] == 3)  # the former, larger exclusion is released
    assert planner.plan(fixed,stamp=1.2)[0] is not None


def test_suppressed_ego_is_stable_and_unknown_does_not_create_road():
    raw = np.full((4,8,16),EGO,np.uint8)
    raw[0,0,0] = 14  # preserve an external person
    fixed = apply_front_robot_prior(raw)
    np.testing.assert_array_equal(apply_front_robot_prior(fixed),fixed)
    assert fixed[0,0,0] == 14
    assert FcnRoadPlanner(LocalPathConfig(),1.6).plan(fixed)[0] is None
    # Complete inference loss is valid unknown data and still cannot start driving.
    assert FcnRoadPlanner(LocalPathConfig(),1.6).plan(apply_front_robot_prior(np.zeros_like(fixed)))[0] is None


def road_logits(_):
    logits = np.zeros((1,21,8,16),np.float32)
    logits[:,3] = 1
    return logits


def test_jetson_and_browser_perception_apply_prior_only_after_four_view_inference():
    from dangjin_fcn import FcnPerception
    panorama = np.zeros((1440,2880,3),np.uint8)
    jetson,views = classify_panorama(panorama,road_logits,input_width=512)
    browser = FcnPerception.__new__(FcnPerception)
    browser.input_width, browser.input_height = 512, 256
    browser.logits = road_logits
    classes,_ = browser.predict_classes(views)
    np.testing.assert_array_equal(classes,jetson)
    assert np.all(classes[0,front_robot_mask()] == EGO)
    assert np.all(classes[1:] == 3)


def test_robot_class_is_visible_in_panorama_preview():
    from dangjin_camera import panorama_mask
    classes = apply_front_robot_prior(np.full((4,8,16),3,np.uint8))
    ego = np.stack([cv2.resize((c == EGO).astype(np.uint8),(480,270),interpolation=cv2.INTER_NEAREST) for c in classes])
    masks = np.zeros((4,270,480),np.uint8)
    pane = panorama_mask(masks,forward_heading_deg=90,ego_masks=ego)
    assert np.any(np.all(pane == (200,70,230),axis=2))
    assert not masks.any()

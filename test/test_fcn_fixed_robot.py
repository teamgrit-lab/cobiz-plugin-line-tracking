"""Only the video-bounded front rig can be ego; it supplies no road evidence."""
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from fcn360_road import (BLIND, EGO, FRAME_ID, FRONT_BLIND, FRONT_ROBOT_ROWS, FcnRoadPlanner, apply_front_robot_prior,
                         classify_panorama, front_robot_mask, observed_classes, read_masks)
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


def test_ros_reader_applies_same_prior_without_mutating_packet_bytes():
    raw = np.full((4,8,16),3,np.uint8)
    payload = raw.tobytes()
    msg = SimpleNamespace(header=SimpleNamespace(frame_id=FRAME_ID),encoding='mono8',
                          height=32,width=16,step=16,data=payload)
    actual = read_masks(msg)
    assert np.all(actual[0,front_robot_mask()] == EGO)
    np.testing.assert_array_equal(actual[1:],raw[1:])
    assert msg.data == payload


def test_suppressed_ego_packet_round_trips_and_unknown_does_not_create_road():
    raw = np.full((4,8,16),EGO,np.uint8)
    raw[0,0,0] = 14  # preserve an external person
    fixed = apply_front_robot_prior(raw)
    msg = SimpleNamespace(header=SimpleNamespace(frame_id=FRAME_ID),encoding='mono8',
                          height=32,width=16,step=16,data=fixed.tobytes())
    np.testing.assert_array_equal(read_masks(msg),fixed)
    assert fixed[0,0,0] == 14
    assert FcnRoadPlanner(LocalPathConfig(),1.6).plan(fixed)[0] is None
    # Complete inference loss is valid unknown data and still cannot start driving.
    msg.data = bytes(512)
    assert FcnRoadPlanner(LocalPathConfig(),1.6).plan(read_masks(msg))[0] is None


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


def test_robot_class_is_visible_in_ros_and_panorama_previews():
    from fcn360_detector import overlay
    from dangjin_camera import panorama_mask
    classes = apply_front_robot_prior(np.full((4,8,16),3,np.uint8))
    views = [np.zeros((256,512,3),np.uint8) for _ in range(4)]
    preview = overlay(views,classes)
    np.testing.assert_array_equal(preview[180,272],(100,35,115))
    assert not np.array_equal(preview[180,512+272],preview[180,272])
    assert not np.array_equal(preview[100,272],preview[180,272])
    ego = np.stack([cv2.resize((c == EGO).astype(np.uint8),(480,270),interpolation=cv2.INTER_NEAREST) for c in classes])
    masks = np.zeros((4,270,480),np.uint8)
    pane = panorama_mask(masks,forward_heading_deg=90,ego_masks=ego)
    assert np.any(np.all(pane == (200,70,230),axis=2))
    assert not masks.any()

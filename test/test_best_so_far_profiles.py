from pathlib import Path
import sys

import numpy as np
import pytest
import torch


TOOLS = Path(__file__).resolve().parents[1] / "tools"
sys.path.insert(0, str(TOOLS))

from best_so_far_runtime import (  # noqa: E402
    DEFAULT_PROFILE,
    R50_MAXIMUM_ROAD_ISLAND_AREA,
    R50_MINIMUM_SIDEWALK_RING_RATIO,
    R50_PROFILE,
    R50_ROAD_LABELS,
    R50_SIDEWALK_LABELS,
    ROAD_ISLAND_ACTIONS,
    SWIN_L_PROFILE,
    BestSoFarConfig,
    BestSoFarSegmenter,
    _changed_pixel_hysteresis_hold_mask,
    resolve_profile,
)


def test_realtime_r50_is_the_default_profile():
    assert DEFAULT_PROFILE == R50_PROFILE
    profile = resolve_profile(DEFAULT_PROFILE)
    assert profile.model_family == "maskformer"
    assert profile.input_height == 360
    assert profile.input_width == 640
    assert profile.precision == "fp16"
    assert profile.temporal_alpha == pytest.approx(0.62)
    assert profile.temporal_hysteresis_margin == pytest.approx(0.0)


def test_realtime_r50_uses_swin_aligned_surface_mapping_and_cleanup():
    assert "Bike Lane" not in R50_ROAD_LABELS
    assert "Parking" not in R50_ROAD_LABELS
    assert "Service Lane" not in R50_ROAD_LABELS
    assert "Bike Lane" in R50_SIDEWALK_LABELS
    assert "Manhole" in R50_SIDEWALK_LABELS
    assert R50_MAXIMUM_ROAD_ISLAND_AREA == 2560
    assert R50_MINIMUM_SIDEWALK_RING_RATIO == pytest.approx(0.10)


def test_swin_l_rollback_profile_is_fully_pinned():
    profile = resolve_profile(SWIN_L_PROFILE)
    assert profile.model_family == "mask2former"
    assert profile.model_id == (
        "facebook/mask2former-swin-large-mapillary-vistas-semantic"
    )
    assert profile.model_revision == "4772b6bf101d91f2534c106dc524d906aeb3c68a"
    assert (profile.input_height, profile.input_width) == (384, 384)
    assert profile.precision == "fp32"
    assert profile.temporal_alpha == pytest.approx(0.62)
    assert profile.temporal_hysteresis_margin == pytest.approx(0.07)


def test_profile_checkpoint_can_be_explicitly_overridden():
    profile = resolve_profile(
        R50_PROFILE,
        model_id="local/model",
        model_revision="abc123",
    )
    assert profile.model_id == "local/model"
    assert profile.model_revision == "abc123"
    assert profile.model_family == "maskformer"


def test_unknown_profile_is_rejected():
    with pytest.raises(ValueError, match="unsupported profile"):
        BestSoFarConfig(profile="unknown").validate()


def test_unknown_road_island_action_is_rejected():
    assert ROAD_ISLAND_ACTIONS == ("drop", "reassign-sidewalk")
    with pytest.raises(ValueError, match="road_island_action"):
        BestSoFarConfig(road_island_action="unknown").validate()


def test_negative_pedestrian_area_road_expansion_is_rejected():
    with pytest.raises(ValueError, match="pedestrian_area_road_expansion"):
        BestSoFarConfig(pedestrian_area_road_expansion=-1).validate()


@pytest.mark.parametrize("ratio", (-0.01, 1.01))
def test_invalid_minimum_sidewalk_ring_ratio_is_rejected(ratio):
    with pytest.raises(ValueError, match="minimum_sidewalk_ring_ratio"):
        BestSoFarConfig(minimum_sidewalk_ring_ratio=ratio).validate()


@pytest.mark.parametrize(
    ("action", "expect_sidewalk"),
    (("drop", False), ("reassign-sidewalk", True)),
)
def test_small_road_island_action(action, expect_sidewalk):
    segmenter = object.__new__(BestSoFarSegmenter)
    segmenter.maximum_road_island_area = 16
    segmenter.minimum_sidewalk_ring_ratio = 0.10
    segmenter.road_island_action = action
    road = np.zeros((12, 12), dtype=bool)
    road[5:7, 5:7] = True
    sidewalk = np.ones((12, 12), dtype=bool)
    sidewalk[5:7, 5:7] = False

    retained_road, retained_sidewalk = segmenter._refine_road_components(
        road,
        sidewalk,
        minimum_area=1,
    )

    assert not retained_road.any()
    assert bool(retained_sidewalk[5:7, 5:7].all()) is expect_sidewalk


def test_road_expands_only_into_adjacent_pedestrian_area():
    segmenter = object.__new__(BestSoFarSegmenter)
    segmenter.pedestrian_area_road_expansion = 1
    segmenter.pedestrian_area_id = 7
    selected = np.full((7, 7), 2, dtype=np.uint8)
    selected[3, 3] = 1
    class_map = np.full((7, 7), 7, dtype=np.int64)
    class_map[2, 2] = 9

    expanded = segmenter._expand_road_into_pedestrian_area(selected, class_map)

    assert expanded[3, 3] == 1
    assert expanded[2, 3] == 1
    assert expanded[2, 2] == 2
    assert expanded[0, 0] == 2


def test_changed_pixel_hysteresis_matches_full_frame_topk():
    generator = np.random.default_rng(17)
    scores = torch.from_numpy(generator.random((7, 16, 20), dtype=np.float32))
    selected = generator.integers(0, 3, size=(16, 20), dtype=np.uint8)
    previous = generator.integers(0, 3, size=(16, 20), dtype=np.uint8)
    margin = 0.07

    top_scores = torch.topk(scores, k=2, dim=0).values
    full_margin = (top_scores[0] - top_scores[1]).numpy()
    expected = (selected != previous) & (full_margin < margin)

    actual = _changed_pixel_hysteresis_hold_mask(
        scores, selected, previous, margin
    )

    assert np.array_equal(actual, expected)

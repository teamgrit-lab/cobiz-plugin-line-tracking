import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import torch

TOOLS = Path(__file__).resolve().parents[1] / "tools"
sys.path.insert(0, str(TOOLS))

from segmentation_runtime import (  # noqa: E402
    DEFAULT_PROFILE,
    R50_MAXIMUM_ROAD_ISLAND_AREA,
    R50_MINIMUM_SIDEWALK_RING_RATIO,
    R50_PROFILE,
    R50_ROAD_LABELS,
    R50_SIDEWALK_LABELS,
    ROAD_ISLAND_ACTIONS,
    SegmentationConfig,
    SegmentationModel,
    _changed_pixel_hysteresis_hold_mask,
    resolve_profile,
)






def test_realtime_r50_preserves_surface_mapping_and_cleanup():
    assert "Bike Lane" not in R50_ROAD_LABELS
    assert "Parking" not in R50_ROAD_LABELS
    assert "Service Lane" not in R50_ROAD_LABELS
    assert "Bike Lane" in R50_SIDEWALK_LABELS
    assert "Manhole" in R50_SIDEWALK_LABELS
    assert R50_MAXIMUM_ROAD_ISLAND_AREA == 2560
    assert R50_MINIMUM_SIDEWALK_RING_RATIO == pytest.approx(0.10)








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
        SegmentationConfig(profile="unknown").validate()








def test_unknown_road_island_action_is_rejected():
    assert ROAD_ISLAND_ACTIONS == ("drop", "reassign-sidewalk")
    with pytest.raises(ValueError, match="road_island_action"):
        SegmentationConfig(road_island_action="unknown").validate()


def test_negative_pedestrian_area_road_expansion_is_rejected():
    with pytest.raises(ValueError, match="pedestrian_area_road_expansion"):
        SegmentationConfig(pedestrian_area_road_expansion=-1).validate()


@pytest.mark.parametrize("ratio", (-0.01, 1.01))
def test_invalid_minimum_sidewalk_ring_ratio_is_rejected(ratio):
    with pytest.raises(ValueError, match="minimum_sidewalk_ring_ratio"):
        SegmentationConfig(minimum_sidewalk_ring_ratio=ratio).validate()


@pytest.mark.parametrize(
    ("action", "expect_sidewalk"),
    (("drop", False), ("reassign-sidewalk", True)),
)
def test_small_road_island_action(action, expect_sidewalk):
    segmenter = object.__new__(SegmentationModel)
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
    segmenter = object.__new__(SegmentationModel)
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

    actual = _changed_pixel_hysteresis_hold_mask(scores, selected, previous, margin)

    assert np.array_equal(actual, expected)










def test_temporal_history_does_not_retain_inference_graphs():
    segmenter = object.__new__(SegmentationModel)
    segmenter.backend = "pytorch"
    segmenter.config = SimpleNamespace(cuda_graph=True)
    segmenter.device = torch.device("cpu")
    segmenter.temporal_alpha = 0.62
    segmenter.temporal_hysteresis_margin = 0.07
    segmenter._previous_scores = None
    segmenter._previous_selected = None
    segmenter.road_ids = [1]
    segmenter.sidewalk_ids = [2]
    segmenter.pedestrian_area_road_expansion = 0
    segmenter.maximum_road_island_area = 0
    parameter = torch.nn.Parameter(torch.zeros((3, 12, 12)))
    modes = []

    def scores(_frame, *, color_order):
        assert color_order == "bgr"
        modes.append(torch.is_inference_mode_enabled())
        return parameter + len(modes)

    segmenter._semantic_scores = scores
    expected = None
    for index in range(1, 65):
        segmenter.segment(np.zeros((12, 12, 3), dtype=np.uint8))
        expected = index if expected is None else 0.62 * index + 0.38 * expected
        history = segmenter._previous_scores
        assert history.grad_fn is None
        assert not history.requires_grad
        assert torch.allclose(history, torch.full_like(history, expected))
    assert all(modes)




def test_default_r50_profile_preserves_pinned_native_input_and_temporal_settings():
    profile = resolve_profile(DEFAULT_PROFILE)
    assert DEFAULT_PROFILE == R50_PROFILE
    assert profile.model_id == "facebook/maskformer-resnet50-vistas"
    assert profile.model_revision == "ae4b8c2590c0a090fc32d5c217d78738a2dd4b19"
    assert (profile.input_height, profile.input_width) == (360, 640)
    assert profile.precision == "fp16"
    assert profile.temporal_alpha == pytest.approx(.62)
    assert profile.temporal_hysteresis_margin == 0


def test_r50_rejects_an_unsupported_inference_backend():
    with pytest.raises(ValueError, match="backend"):
        SegmentationConfig(backend="tensorrt").validate()

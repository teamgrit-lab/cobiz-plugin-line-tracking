"""Compare accelerator postprocessing with the retained CPU round-trip path."""

import sys
from pathlib import Path

import numpy as np
import pytest
import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

from best_so_far_runtime import BestSoFarSegmenter  # noqa: E402


@pytest.fixture(params=["cpu", "cuda", "mps"])
def device(request):
    name = request.param
    if name == "cuda" and not torch.cuda.is_available():
        pytest.skip("CUDA unavailable")
    if name == "mps" and not torch.backends.mps.is_available():
        pytest.skip("MPS unavailable")
    return torch.device(name)


def make_segmenter(device, *, margin=0.07, expansion=0, action="drop"):
    segmenter = object.__new__(BestSoFarSegmenter)
    segmenter.backend = "tensorrt"
    segmenter.device = device
    segmenter.temporal_alpha = 0.62
    segmenter.temporal_hysteresis_margin = margin
    # Also check that sidewalk wins when aggregation labels overlap.
    segmenter.road_ids = [1, 4]
    segmenter.sidewalk_ids = [2, 3, 4]
    segmenter.pedestrian_area_id = 3
    segmenter.pedestrian_area_road_expansion = expansion
    segmenter.maximum_road_island_area = 100 if action == "reassign-sidewalk" else 0
    segmenter.minimum_sidewalk_ring_ratio = 0.1
    segmenter.road_island_action = action
    segmenter.reset()
    return segmenter


@pytest.mark.parametrize("dtype", [torch.float16, torch.float32])
@pytest.mark.parametrize("margin,expansion", [(0.0, 0), (0.07, 0), (0.07, 2)])
@pytest.mark.parametrize("action", ["drop", "reassign-sidewalk"])
def test_frame_sequence_matches_cpu_round_trip(device, dtype, margin, expansion, action):
    actual = make_segmenter(device, margin=margin, expansion=expansion, action=action)
    reference = make_segmenter(device, margin=margin, expansion=expansion, action=action)
    # Exercise the accelerator implementation even on CPU-only CI machines;
    # run the original CPU round trips on the same score device for comparison.
    actual._cpu_selected_mask = actual._accelerator_selected_mask
    reference._accelerator_selected_mask = reference._cpu_selected_mask
    generator = torch.Generator().manual_seed(421)
    source = torch.rand((6, 32, 48), generator=generator) * 0.08
    source[2] += 0.3
    source[1, 5:13, 5:13] += 0.8  # Road island: retain or reassign to sidewalk.
    source[1, 20:23, 20:23] += 0.8  # Small component removed on CPU.
    source[3, :, 30:] += 0.4  # Pedestrian area, with a road at the image edge.
    source[1, 0:10, 38:46] += 0.8
    frame = np.zeros((32, 48, 3), dtype=np.uint8)
    buffer = source.to(device=device, dtype=dtype)
    for index in range(6):
        # Reuse a TensorRT-style output buffer; postprocessing must not alter it
        # or retain it as temporal state that is overwritten next frame.
        next_scores = source.clone()
        if index:
            next_scores += torch.rand(source.shape, generator=generator) * 0.15
            next_scores[1, :, :24] += index * 0.10
        buffer.copy_(next_scores)
        before = buffer.clone()
        actual._semantic_scores = lambda _frame, **_kwargs: buffer
        reference._semantic_scores = actual._semantic_scores
        result = actual.segment(frame)
        expected = reference.segment(frame)
        np.testing.assert_array_equal(result.selected_mask, expected.selected_mask)
        assert result.hysteresis_hold_ratio == expected.hysteresis_hold_ratio
        assert result.road_area_ratio == expected.road_area_ratio
        assert result.sidewalk_area_ratio == expected.sidewalk_area_ratio
        assert result.selected_mask.dtype == np.uint8
        assert result.selected_mask.flags.c_contiguous
        assert torch.equal(buffer, before)
        assert torch.equal(actual._previous_scores, reference._previous_scores)
    actual.reset()
    reference.reset()
    assert actual._surface_lookup is None
    np.testing.assert_array_equal(
        actual.segment(frame).selected_mask, reference.segment(frame).selected_mask
    )


@pytest.mark.parametrize("dtype", [torch.float16, torch.float32])
def test_ties_and_threshold_rounding_match_cpu(device, dtype):
    segmenter = make_segmenter(device)
    # Both representable FP16 differences straddling a threshold that rounds
    # down when converted to half. Comparing in FP16 would change the result.
    segmenter.temporal_hysteresis_margin = 0.06998
    scores = torch.zeros((6, 1, 5), dtype=dtype, device=device)
    scores[1] = torch.tensor([0.5, 0.5, 0.0699462890625, 0.07000732421875, 0.8])
    scores[2, 0, :2] = 0.5
    segmenter._previous_selected = np.full((1, 5), 2, dtype=np.uint8)
    with torch.inference_mode():
        expected, expected_ratio = segmenter._cpu_selected_mask(scores)
        actual, actual_ratio = segmenter._accelerator_selected_mask(scores)
        np.testing.assert_array_equal(actual, expected)
        assert actual_ratio == expected_ratio == 0.6
        segmenter.temporal_hysteresis_margin = 0.0
        actual, _ = segmenter._accelerator_selected_mask(scores)
        assert actual[0, 0] == 1  # argmax's first class on a tie.


def test_hysteresis_uses_history_after_cpu_component_removal(device):
    segmenter = make_segmenter(device)
    segmenter.temporal_alpha = 1.0
    segmenter._cpu_selected_mask = segmenter._accelerator_selected_mask
    scores = torch.zeros((6, 16, 16), device=device)
    scores[2] = 0.9
    scores[1, 5:8, 5:8] = 1.0
    segmenter._semantic_scores = lambda _frame, **_kwargs: scores
    frame = np.zeros((16, 16, 3), dtype=np.uint8)
    first = segmenter.segment(frame)
    assert np.all(first.selected_mask[5:8, 5:8] == 0)
    scores[1, 5:8, 5:8] = 0.92
    second = segmenter.segment(frame)
    # The removed island must stay background under low-confidence hysteresis.
    # Pre-morphology history would see unchanged road and report no hold.
    assert np.all(second.selected_mask[5:8, 5:8] == 0)
    assert second.hysteresis_hold_ratio == 9 / 256


def test_unchanged_labels_do_not_count_as_hysteresis_holds(device):
    segmenter = make_segmenter(device)
    scores = torch.zeros((6, 4, 5), device=device)
    scores[1] = 0.5
    scores[2] = 0.49
    segmenter._previous_selected = np.ones((4, 5), dtype=np.uint8)
    with torch.inference_mode():
        selected, ratio = segmenter._accelerator_selected_mask(scores)
    np.testing.assert_array_equal(selected, segmenter._previous_selected)
    assert ratio == 0.0

"""Vegetation uses one current inference, independently of surface history."""

import numpy as np
import pytest
import torch

from test_accelerator_postprocess import device, make_segmenter


@pytest.mark.parametrize("backend", ["pytorch", "tensorrt"])
def test_current_vegetation_mask_has_no_ema_ghost_and_preserves_surface_labels(device, backend):
    segmenter = make_segmenter(device)
    segmenter.backend = backend
    segmenter.vegetation_id = 5
    segmenter.temporal_alpha = 0.1
    reference = make_segmenter(device)
    reference.backend = backend
    reference.temporal_alpha = segmenter.temporal_alpha
    scores = torch.zeros((6, 32, 48), device=device)
    scores[1] = 1
    scores[5, 5:15, 5:15] = 10
    calls = []

    def semantic(_frame, **_kwargs):
        calls.append(1)
        return scores

    segmenter._semantic_scores = semantic
    reference._semantic_scores = lambda *_args, **_kwargs: scores
    frame = np.zeros((32, 48, 3), np.uint8)
    first = segmenter.segment(frame, detect_vegetation=True)
    assert len(calls) == 1
    assert first.vegetation_mask.sum() == 100
    np.testing.assert_array_equal(first.selected_mask, reference.segment(frame).selected_mask)
    scores = scores.clone()
    scores[5] = 0
    second = segmenter.segment(frame, detect_vegetation=True)
    assert len(calls) == 2
    assert not second.vegetation_mask.any()
    assert segmenter._previous_scores[5, 6, 6] > segmenter._previous_scores[1, 6, 6]
    np.testing.assert_array_equal(second.selected_mask, reference.segment(frame).selected_mask)
    assert segmenter.segment(frame).vegetation_mask is None


def test_weak_or_missing_vegetation_label_does_not_supply_bias_evidence():
    segmenter = make_segmenter(torch.device("cpu"))
    scores = torch.ones((6, 32, 48))
    scores[5] = 1.1  # Winning class, but <0.5 of the total semantic score.
    segmenter._semantic_scores = lambda *_args, **_kwargs: scores
    frame = np.zeros((32, 48, 3), np.uint8)
    assert segmenter.segment(frame, detect_vegetation=True).vegetation_mask is None
    segmenter.vegetation_id = 5
    assert not segmenter.segment(frame, detect_vegetation=True).vegetation_mask.any()

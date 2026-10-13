"""Pole extraction shares existing scores and never reuses surface EMA."""

import numpy as np
import pytest
import torch

from test_accelerator_postprocess import make_segmenter


@pytest.mark.parametrize("backend", ["pytorch", "tensorrt"])
def test_current_frame_pole_mask_excludes_lamps_and_utility_poles_without_ghosts(backend):
    segmenter = make_segmenter(torch.device("cpu"))
    segmenter.backend = backend
    segmenter.pole_id = 45
    segmenter.temporal_alpha = 0.1
    scores = torch.zeros((48, 32, 48))
    scores[1] = 1
    scores[45, 5:15, 5:15] = 10
    scores[44, 5:15, 25:35] = 10  # A lamp is not a pole target.
    scores[47, 20:30, 5:15] = 10  # Utility Pole is a different class.
    calls = []

    def semantic(_frame, **_kwargs):
        calls.append(1)
        return scores

    segmenter._semantic_scores = semantic
    frame = np.zeros((32, 48, 3), np.uint8)
    result = segmenter.segment(frame, detect_poles=True)
    assert len(calls) == 1
    expected = np.zeros((32, 48), bool)
    expected[5:15, 5:15] = True
    np.testing.assert_array_equal(result.pole_mask, expected)
    assert set(np.unique(result.selected_mask)) <= {0, 1, 2}
    scores = scores.clone()
    scores[45] = 0
    result = segmenter.segment(frame, detect_poles=True)
    assert len(calls) == 2
    assert not result.pole_mask.any()
    assert segmenter._previous_scores[45, 6, 6] > segmenter._previous_scores[1, 6, 6]
    assert segmenter.segment(frame).pole_mask is None


def test_weak_pole_scores_are_filtered_before_components_are_confirmed():
    segmenter = make_segmenter(torch.device("cpu"))
    segmenter.pole_id = 45
    scores = torch.ones((48, 32, 48))
    scores[45] = 1.1  # Winner, but below 50% of the total semantic score.
    segmenter._semantic_scores = lambda *_args, **_kwargs: scores
    result = segmenter.segment(np.zeros((32, 48, 3), np.uint8), detect_poles=True)
    assert not result.pole_mask.any()


def test_lamp_label_cannot_replace_a_missing_pole_label():
    segmenter = make_segmenter(torch.device("cpu"))
    segmenter.pole_id = None
    segmenter.street_light_id = 44
    scores = torch.zeros((48, 32, 48))
    scores[44] = 1
    segmenter._semantic_scores = lambda *_args, **_kwargs: scores
    with pytest.raises(RuntimeError, match="missing Pole label"):
        segmenter.segment(np.zeros((32, 48, 3), np.uint8), detect_poles=True)

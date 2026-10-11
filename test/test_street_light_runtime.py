"""Street Light extraction shares existing scores and never reuses surface EMA."""

import numpy as np
import pytest
import torch

from test_accelerator_postprocess import make_segmenter


@pytest.mark.parametrize("backend", ["pytorch", "tensorrt"])
def test_current_frame_light_mask_does_not_change_surface_labels_or_keep_ghosts(backend):
    segmenter = make_segmenter(torch.device("cpu"))
    segmenter.backend = backend
    segmenter.street_light_id = 5
    segmenter.temporal_alpha = 0.1
    scores = torch.zeros((6, 32, 48))
    scores[1] = 1
    scores[5, 5:15, 5:15] = 10
    calls = []

    def semantic(_frame, **_kwargs):
        calls.append(1)
        return scores

    segmenter._semantic_scores = semantic
    frame = np.zeros((32, 48, 3), np.uint8)
    result = segmenter.segment(frame, detect_street_lights=True)
    assert len(calls) == 1
    assert result.street_light_mask.sum() == 100
    assert set(np.unique(result.selected_mask)) <= {0, 1, 2}
    scores = scores.clone()
    scores[5] = 0
    result = segmenter.segment(frame, detect_street_lights=True)
    assert len(calls) == 2
    assert not result.street_light_mask.any()
    assert segmenter._previous_scores[5, 6, 6] > segmenter._previous_scores[1, 6, 6]
    assert segmenter.segment(frame).street_light_mask is None


def test_weak_light_scores_are_filtered_before_components_are_confirmed():
    segmenter = make_segmenter(torch.device("cpu"))
    segmenter.street_light_id = 5
    scores = torch.ones((6, 32, 48))
    scores[5] = 1.1  # Winner, but only 18% of total semantic score.
    segmenter._semantic_scores = lambda *_args, **_kwargs: scores
    result = segmenter.segment(np.zeros((32, 48, 3), np.uint8), detect_street_lights=True)
    assert not result.street_light_mask.any()

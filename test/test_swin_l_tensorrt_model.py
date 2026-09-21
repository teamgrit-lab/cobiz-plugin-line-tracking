import sys
from pathlib import Path
from types import SimpleNamespace

import torch

TOOLS = Path(__file__).resolve().parents[1] / "tools"
sys.path.insert(0, str(TOOLS))

from swin_l_tensorrt_model import SwinLSemanticScores


class FakeMask2Former(torch.nn.Module):
    def forward(self, *, pixel_values, return_dict):
        assert tuple(pixel_values.shape) == (1, 3, 4, 6)
        assert return_dict is True
        return SimpleNamespace(
            class_queries_logits=torch.tensor([[[4.0, -4.0, -8.0], [-4.0, 4.0, -8.0]]]),
            masks_queries_logits=torch.tensor(
                [[[[4.0, -4.0], [4.0, -4.0]], [[-4.0, 4.0], [-4.0, 4.0]]]]
            ),
        )


def test_export_wrapper_returns_semantic_scores_at_fixed_evaluation_size():
    wrapper = SwinLSemanticScores(
        FakeMask2Former(), evaluation_height=4, evaluation_width=6
    )

    scores = wrapper(torch.zeros((1, 3, 4, 6)))

    assert tuple(scores.shape) == (2, 4, 6)
    assert torch.all(scores[0, :, 0] > scores[1, :, 0])
    assert torch.all(scores[1, :, -1] > scores[0, :, -1])

"""Export-stable Swin-L Mask2Former semantic-score wrapper."""

from __future__ import annotations

from typing import Any

import torch
from torch.nn import functional


def tensor_only_swin_stage_outputs(
    output: Any,
) -> tuple[torch.Tensor, torch.Tensor]:
    """Drop the unused optional attention output before TRT serialization."""

    if not isinstance(output, (tuple, list)):
        raise TypeError("Swin stage output must be a tuple or list")
    if len(output) != 3:
        raise RuntimeError("Swin stage must return three outputs")
    hidden_states, reshaped_hidden_states, attention = output
    if not isinstance(hidden_states, torch.Tensor) or not isinstance(
        reshaped_hidden_states, torch.Tensor
    ):
        raise TypeError("Swin stage hidden-state outputs must be tensors")
    if attention is not None:
        raise RuntimeError("TensorRT Swin stages do not support attention outputs")
    return hidden_states, reshaped_hidden_states


def restore_swin_stage_outputs(
    output: Any,
) -> tuple[torch.Tensor, torch.Tensor, None]:
    """Restore the Transformers Swin stage tuple around a TRT module."""

    if not isinstance(output, (tuple, list)):
        raise TypeError("serialized TensorRT Swin stage output must be a tuple or list")
    if len(output) != 2:
        raise RuntimeError("serialized TensorRT Swin stage must return two tensors")
    hidden_states, reshaped_hidden_states = output
    if not isinstance(hidden_states, torch.Tensor) or not isinstance(
        reshaped_hidden_states, torch.Tensor
    ):
        raise TypeError("serialized TensorRT Swin stage outputs must be tensors")
    return hidden_states, reshaped_hidden_states, None


class SwinLSemanticScores(torch.nn.Module):
    """Return the exact score tensor consumed by the retained temporal pipeline."""

    def __init__(
        self,
        model: torch.nn.Module,
        *,
        evaluation_height: int = 360,
        evaluation_width: int = 640,
    ) -> None:
        super().__init__()
        if evaluation_height <= 0 or evaluation_width <= 0:
            raise ValueError("evaluation dimensions must be positive")
        self.model = model
        self.evaluation_height = evaluation_height
        self.evaluation_width = evaluation_width

    def forward(self, pixel_values: torch.Tensor) -> torch.Tensor:
        outputs = self.model(pixel_values=pixel_values, return_dict=True)
        class_probabilities = outputs.class_queries_logits.softmax(dim=-1)[..., :-1]
        mask_probabilities = outputs.masks_queries_logits.sigmoid()
        scores = torch.einsum(
            "bqc,bqhw->bchw",
            class_probabilities,
            mask_probabilities,
        )
        scores = functional.interpolate(
            scores,
            size=(self.evaluation_height, self.evaluation_width),
            mode="bilinear",
            align_corners=False,
        )
        return scores[0]

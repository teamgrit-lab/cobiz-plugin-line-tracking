"""Export-stable Swin-L Mask2Former semantic-score wrapper."""

from __future__ import annotations

import torch
from torch.nn import functional


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

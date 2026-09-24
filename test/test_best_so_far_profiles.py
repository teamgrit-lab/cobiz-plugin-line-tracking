import sys
from pathlib import Path
from types import SimpleNamespace

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
    SWIN_L_ASPECT_FP16_PROFILE,
    SWIN_L_ASPECT_PROFILE,
    SWIN_L_ASPECT_QUALITY_PROFILE,
    SWIN_L_PROFILE,
    BestSoFarConfig,
    BestSoFarSegmenter,
    _changed_pixel_hysteresis_hold_mask,
    resolve_profile,
)


def test_swin_l_aspect_fp16_is_the_selected_default_profile():
    assert DEFAULT_PROFILE == SWIN_L_ASPECT_FP16_PROFILE
    profile = resolve_profile(DEFAULT_PROFILE)
    assert profile.model_family == "mask2former"
    assert profile.input_height == 224
    assert profile.input_width == 384
    assert profile.precision == "fp16"
    assert profile.temporal_alpha == pytest.approx(0.62)
    assert profile.temporal_hysteresis_margin == pytest.approx(0.07)


def test_swin_l_aspect_fp16_changes_only_name_and_precision():
    fp32 = resolve_profile(SWIN_L_ASPECT_PROFILE)
    fp16 = resolve_profile(SWIN_L_ASPECT_FP16_PROFILE)

    assert fp16.name == "swin-l-aspect-224x384-fp16"
    assert fp16.precision == "fp16"
    assert fp16.model_family == fp32.model_family
    assert fp16.model_id == fp32.model_id
    assert fp16.model_revision == fp32.model_revision
    assert (fp16.input_height, fp16.input_width) == (
        fp32.input_height,
        fp32.input_width,
    )
    assert fp16.temporal_alpha == fp32.temporal_alpha
    assert fp16.temporal_hysteresis_margin == fp32.temporal_hysteresis_margin


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


def test_swin_l_aspect_profile_preserves_checkpoint_and_temporal_settings():
    baseline = resolve_profile(SWIN_L_PROFILE)
    aspect = resolve_profile(SWIN_L_ASPECT_PROFILE)
    assert aspect.model_family == baseline.model_family
    assert aspect.model_id == baseline.model_id
    assert aspect.model_revision == baseline.model_revision
    assert (aspect.input_height, aspect.input_width) == (224, 384)
    assert aspect.precision == baseline.precision
    assert aspect.temporal_alpha == baseline.temporal_alpha
    assert aspect.temporal_hysteresis_margin == baseline.temporal_hysteresis_margin


def test_swin_l_quality_profile_preserves_checkpoint_and_temporal_settings():
    baseline = resolve_profile(SWIN_L_PROFILE)
    quality = resolve_profile(SWIN_L_ASPECT_QUALITY_PROFILE)
    assert quality.model_family == baseline.model_family
    assert quality.model_id == baseline.model_id
    assert quality.model_revision == baseline.model_revision
    assert (quality.input_height, quality.input_width) == (448, 768)
    assert quality.precision == baseline.precision
    assert quality.temporal_alpha == baseline.temporal_alpha
    assert quality.temporal_hysteresis_margin == baseline.temporal_hysteresis_margin


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


def test_tensorrt_backend_requires_fixed_artifact_paths():
    with pytest.raises(ValueError, match="engine and manifest"):
        BestSoFarConfig(backend="tensorrt").validate()


def test_tensorrt_backend_accepts_pinned_fp16_swin_profile():
    BestSoFarConfig(
        backend="tensorrt",
        tensorrt_engine_path="model.plan",
        tensorrt_manifest_path="model.plan.json",
    ).validate()


def test_tensorrt_backend_rejects_fp32_profile():
    with pytest.raises(ValueError, match="FP16 Mask2Former"):
        BestSoFarConfig(
            profile=SWIN_L_ASPECT_PROFILE,
            backend="tensorrt",
            tensorrt_engine_path="model.plan",
            tensorrt_manifest_path="model.plan.json",
        ).validate()


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

    actual = _changed_pixel_hysteresis_hold_mask(scores, selected, previous, margin)

    assert np.array_equal(actual, expected)


class _FakeMask2FormerProcessor:
    def __init__(self, scores):
        self.scores = scores

    def __call__(self, *, images, return_tensors):
        assert images.shape == (4, 6, 3)
        assert return_tensors == "pt"
        return {"pixel_values": torch.ones((1, 3, 4, 6), dtype=torch.float32)}

    def post_process_semantic_segmentation(
        self, outputs, *, target_sizes, return_segmentation_scores
    ):
        assert outputs == "model-output"
        assert target_sizes == [(4, 6)]
        assert return_segmentation_scores is True
        return [{"segmentation_scores": self.scores}]


class _FakeMask2FormerModel:
    def __call__(self, **inputs):
        assert inputs["pixel_values"].dtype == torch.float16
        return "model-output"


def test_swin_l_fp16_scores_remain_on_accelerator_in_native_dtype():
    segmenter = object.__new__(BestSoFarSegmenter)
    segmenter.profile = SimpleNamespace(model_family="mask2former")
    segmenter.config = SimpleNamespace(evaluation_height=4, evaluation_width=6)
    segmenter.device = torch.device("cpu")
    segmenter.use_fp16 = True
    expected = torch.ones((3, 4, 6), dtype=torch.float16)
    segmenter.processor = _FakeMask2FormerProcessor(expected)
    segmenter.model = _FakeMask2FormerModel()

    actual = segmenter._semantic_scores(np.zeros((4, 6, 3), dtype=np.uint8))

    assert actual.data_ptr() == expected.data_ptr()
    assert actual.dtype == torch.float16
    assert actual.device == expected.device


def test_swin_l_fp32_rollback_scores_stay_float32_on_cpu():
    segmenter = object.__new__(BestSoFarSegmenter)
    segmenter.profile = SimpleNamespace(model_family="mask2former")
    segmenter.config = SimpleNamespace(evaluation_height=4, evaluation_width=6)
    segmenter.device = torch.device("cpu")
    segmenter.use_fp16 = False
    source = torch.ones((3, 4, 6), dtype=torch.float16)
    segmenter.processor = _FakeMask2FormerProcessor(source)
    segmenter.model = lambda **_inputs: "model-output"

    actual = segmenter._semantic_scores(np.zeros((4, 6, 3), dtype=np.uint8))

    assert actual.dtype == torch.float32
    assert actual.device.type == "cpu"


def test_temporal_history_does_not_retain_hybrid_decoder_graphs():
    segmenter = object.__new__(BestSoFarSegmenter)
    segmenter.backend = "tensorrt"
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


def test_native_rgb_matches_bgr_masks_and_skips_unused_tensorrt_inputs(monkeypatch):
    import cv2

    segmenter = object.__new__(BestSoFarSegmenter)
    segmenter.backend = "tensorrt"
    segmenter.device = torch.device("cpu")
    segmenter.use_fp16 = True
    segmenter.temporal_alpha = 0.62
    segmenter.temporal_hysteresis_margin = 0.07
    segmenter.road_ids = [1]
    segmenter.sidewalk_ids = [2]
    segmenter.pedestrian_area_road_expansion = 0
    segmenter.maximum_road_island_area = 0
    segmenter.reset()
    seen_rgb = []

    class UnusedMask:
        def is_floating_point(self):
            pytest.fail("TensorRT must not move pixel_mask to the accelerator")

    def processor(*, images, return_tensors):
        assert return_tensors == "pt"
        seen_rgb.append(images.copy())
        return {
            "pixel_values": torch.from_numpy(images.copy()).permute(2, 0, 1)[None].float(),
            "pixel_mask": UnusedMask(),
        }

    segmenter.processor = processor
    segmenter.tensorrt_backend = SimpleNamespace(semantic_scores=lambda values: values[0])
    rgb = np.zeros((12, 24, 3), dtype=np.uint8)
    rgb[:, :12, 1] = 200
    rgb[:, 12:, 2] = 250
    expected = segmenter.segment(cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
    segmenter.reset()

    def unexpected_color_conversion(*_args, **_kwargs):
        pytest.fail("native RGB must reach the processor without a color conversion")

    monkeypatch.setattr(cv2, "cvtColor", unexpected_color_conversion)
    actual = segmenter.segment(rgb, color_order="rgb")

    np.testing.assert_array_equal(seen_rgb[0], rgb)
    np.testing.assert_array_equal(seen_rgb[1], rgb)
    np.testing.assert_array_equal(actual.selected_mask, expected.selected_mask)
    assert set(np.unique(actual.selected_mask)) == {1, 2}

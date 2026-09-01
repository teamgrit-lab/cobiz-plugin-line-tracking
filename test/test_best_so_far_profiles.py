from pathlib import Path
import sys

import pytest


TOOLS = Path(__file__).resolve().parents[1] / "tools"
sys.path.insert(0, str(TOOLS))

from best_so_far_runtime import (  # noqa: E402
    DEFAULT_PROFILE,
    R50_MAXIMUM_ROAD_ISLAND_AREA,
    R50_MINIMUM_SIDEWALK_RING_RATIO,
    R50_PROFILE,
    R50_ROAD_LABELS,
    R50_SIDEWALK_LABELS,
    SWIN_L_PROFILE,
    BestSoFarConfig,
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

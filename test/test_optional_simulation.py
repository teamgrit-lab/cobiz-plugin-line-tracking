"""Real-data preparation and camera contracts work without local scene assets."""
import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from surround_camera import load_profile, profile_sha256
from training_geometry import load_simulation_world
from train_front_connectome import entries


def test_real_front_entries_do_not_require_simulation_or_cached_synthetic_data(tmp_path):
    (tmp_path / 'splits').mkdir()
    (tmp_path / 'samples.jsonl').write_text(json.dumps(
        {'id': 'frame', 'camera': {'image': 'frame.png'}}) + '\n')
    (tmp_path / 'splits/policy.json').write_text(json.dumps({'assignments': {'session': 'train'}}))
    (tmp_path / 'pseudo_labels.jsonl').write_text(json.dumps(
        {'id': 'frame', 'session': 'session', 'labels': {'road': {'index': 0}},
         'estimated_command': {'linear_x_mps': .2, 'angular_z_radps': 0, 'stop': False}}) + '\n')
    rows = entries(tmp_path)
    assert len(rows) == 1
    assert rows[0]['source'] == 'real_pseudo'
    assert rows[0]['image'] == str(tmp_path / 'frame.png')
    with pytest.raises(ValueError, match='--simulation-dir'):
        entries(tmp_path, simulation_dir=tmp_path / 'missing-scene')


def test_optional_geometry_fails_with_an_actionable_missing_asset_message(tmp_path):
    with pytest.raises(ValueError, match='--simulation-dir'):
        load_simulation_world(tmp_path)


def test_tracked_camera_profile_keeps_the_deployed_projection_identity():
    # The identity is pinned by existing trained model packages, independently
    # of the on-disk location of the JSON configuration.
    profile = load_profile()
    assert profile['id'] == 'video91-front90-v1'
    assert profile_sha256(profile) == '77c4d25f4e4da0555369bd6ed607011a22b969c8a67974cb2ae32ff278318cdf'

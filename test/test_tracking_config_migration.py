"""Deployment migrations retain model identity and reject ambiguous settings."""
import subprocess
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from migrate_tracking_config import migrate, validate_environment


def test_model_pin_and_controller_values_survive_migration():
    original = (
        'SWIN_L_PROFILE=r50-finetuned-fp16-640x360\n'
        'SWIN_L_MODEL_ID="/models/custom/best"\n'
        'SWIN_L_CHECKPOINT_SHA256=' + 'a' * 64 + '\n'
        'SWIN_L_ENGINE_DIR=./models\nSWIN_L_BACKEND=pytorch\n'
        'SWIN_L_TRT_AUTO_BUILD=false\nLINE_TRACKING_MAX_FORWARD_MPS=0.42\n'
        'SWIN_L_METRICS_TOPIC=/line_tracking/swin_l/metrics\n')
    updated = migrate(original)
    assert 'LINE_TRACKING_PROFILE=r50-finetuned-fp16-640x360' in updated
    assert 'LINE_TRACKING_MODEL_ID="/models/custom/best"' in updated
    assert 'LINE_TRACKING_CHECKPOINT_SHA256=' + 'a' * 64 in updated
    assert 'LINE_TRACKING_MODEL_DIR=./models' in updated
    assert 'LINE_TRACKING_MAX_FORWARD_MPS=0.42' in updated
    assert 'LINE_TRACKING_METRICS_TOPIC=/line_tracking/metrics' in updated
    assert 'TRT_AUTO_BUILD' not in updated
    assert migrate(updated) == updated


def test_migration_rejects_conflicting_old_and_new_keys():
    with pytest.raises(ValueError, match='Duplicate destination'):
        migrate('SWIN_L_PROFILE=r50-fp16-640x360\nLINE_TRACKING_PROFILE=r50-fp16-640x360\n')
    with pytest.raises(ValueError, match='supported R50'):
        migrate('SWIN_L_PROFILE=unsupported-model\n')
    with pytest.raises(ValueError, match='migrate_tracking_config'):
        validate_environment({'SWIN_L_DEVICE': 'cuda'})


def test_removed_stop_settings_are_purged_without_changing_remaining_controls():
    retired = ('STOP_ON_CAMERA_TIMESTAMP_INVALID', 'STOP_ON_PATH_UNAVAILABLE',
               'STOP_ON_LOW_CONFIDENCE', 'STOP_ON_UNSAFE_TIMEOUT', 'STOP_ON_LATERAL_TARGET',
               'UNSAFE_TIMEOUT_SEC', 'MAX_TARGET_HEADING_DEG')
    retained = 'LINE_TRACKING_STOP_ON_CAMERA_STALE=true\nLINE_TRACKING_MAX_FORWARD_MPS=0.80\n'
    original = ''.join('LINE_TRACKING_' + key + '=true\n' for key in retired) + retained
    assert migrate(original) == retained


def test_write_creates_private_backup_and_keeps_output_free_of_values(tmp_path):
    env = tmp_path / '.env'
    original = 'SWIN_L_MODEL_ID=/models/private/best\nSWIN_L_DEVICE=cuda\n'
    env.write_text(original)
    script = Path(__file__).resolve().parents[1] / 'tools/migrate_tracking_config.py'
    result = subprocess.run([sys.executable, str(script), '--env-file', str(env), '--write'],
                            capture_output=True, text=True, check=True)
    backup = env.with_name('.env.before-line-tracking')
    assert backup.read_text() == original
    assert backup.stat().st_mode & 0o777 == 0o600
    assert env.stat().st_mode & 0o777 == 0o600
    assert '/models/private' not in result.stdout
    assert 'LINE_TRACKING_MODEL_ID=' in env.read_text()

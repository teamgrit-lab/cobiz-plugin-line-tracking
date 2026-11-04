"""360 model pairing, projection and physical command contracts."""
import json
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
from connectome360_client import Connectome360Segmenter
from connectome360_runtime import package_config, sha256
from surround_camera import load_profile, profile_sha256
from swin_l_drive_control import DriveDecision
from road_test_support import PIN, details

ARGS = ('--connectome360-url', 'http://127.0.0.1:8093/v1/direct-road',
        '--connectome360-sha256', PIN, '--path-mask-class', '1', '--path-planner', 'local')


def test_model_manifest_binds_all_weights_and_the_projection(tmp_path):
    for name in ['flyvis_encoder.pt', 'road/road-readout.pt', 'policy/drive-readout.pt']:
        p = tmp_path/name
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_bytes(name.encode())
    config = dict(schema='connectome360.camera_only.pair.v1', simulation_max_speed_mps=10.,
        input_contract=dict(panorama_shape_wh=[2880, 1440], camera_views=['front', 'left', 'rear', 'right'],
            view_shape_wh=[480, 270], forward_heading_deg=90, actor_input_chw=[4, 32, 64], actor_crop_rows=[135, 257]),
        camera_profile_sha256=profile_sha256(load_profile()),
        encoder_sha256=sha256(tmp_path/'flyvis_encoder.pt'), road_sha256=sha256(tmp_path/'road/road-readout.pt'),
        policy_sha256=sha256(tmp_path/'policy/drive-readout.pt'))
    (tmp_path/'model.json').write_text(json.dumps(config))
    pin = sha256(tmp_path/'model.json')
    assert package_config(tmp_path, pin) == config
    (tmp_path/'policy/drive-readout.pt').write_bytes(b'wrong policy')
    with pytest.raises(ValueError, match='weight SHA-256'): package_config(tmp_path, pin)
    with pytest.raises(ValueError, match='model.json SHA-256'): package_config(tmp_path, '0'*64)



@pytest.mark.parametrize('command', [
    dict(vx=float('nan'), vy=0., yaw_rate=0., reason='neural_drive'),
    dict(vx=10.1, vy=0., yaw_rate=0., reason='neural_drive'),
    dict(vx=.1, vy=.1, yaw_rate=0., reason='neural_drive'),
    dict(vx=.1, vy=0., yaw_rate=.4, reason='neural_drive'),
    dict(vx=.1, vy=0., yaw_rate=0., reason='neural_wait'),
])
def test_invalid_command_never_gets_clamped_into_a_valid_move(command):
    client = Connectome360Segmenter.__new__(Connectome360Segmenter)
    client.details = details()
    with pytest.raises(ValueError): client.decode_command(command)
    assert client.decode_command(dict(vx=0., vy=0., yaw_rate=0., reason='neural_wait')) == DriveDecision.stop('neural_wait')



def test_float_roundoff_does_not_reject_a_command_on_the_transport_limit():
    client = Connectome360Segmenter.__new__(Connectome360Segmenter)
    client.details = details()
    for vx in np.random.default_rng(42).uniform(.8, 10, 1000):
        c = client.decode_command(dict(vx=float(vx), vy=0., yaw_rate=.35, reason='neural_drive'))
        assert 0 <= c.vx <= .8 and 0 <= c.yaw_rate <= .35
        assert c.yaw_rate/c.vx == pytest.approx(.35/vx)



def test_rejects_multiple_road_backends(monkeypatch):
    from test_apriltag_task_stop_ros import debug
    monkeypatch.setattr(debug, 'ENV', {})
    for extra in [('--front-connectome-url', 'http://127.0.0.1:8092/v1/front-road'),
                  ('--fcn-sha256', 'a'*64), ('--path-mask-class', '0')]:
        with pytest.raises(SystemExit): debug.parse_args(['task-drive', *ARGS, *extra])

"""Weak-target authorization and agreement between training and runtime limits."""
import json
from pathlib import Path
import sys

import numpy as np
import pytest
import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
from drive_control import DriveDecision
from front_connectome_client import limit_front_command
from finetune_r50_360_policy import read_rows, NORMALIZATION
from r50_policy_finetuning import physical_action
from r50_policy_package import sha256


def test_training_limits_match_runtime_and_preserve_finite_gradients():
    action = torch.tensor([[.8,.3],[.05,.5],[.5,-1.],[.08,0.],[.03,-.4]], requires_grad=True)
    result = physical_action(action)
    expected = []
    for v, yaw in action.detach().numpy():
        command = limit_front_command(DriveDecision(float(v*10),0.,float(yaw*.35),'neural_drive'),.8,.18)
        expected.append([command.vx,command.yaw_rate])
    np.testing.assert_allclose(result.detach().numpy(), expected, atol=1e-6)
    result.sum().backward()
    assert torch.isfinite(action.grad).all()


def test_authorized_weak_labels_remain_unverified_and_exclusions_cannot_enter(tmp_path):
    root=tmp_path/'dataset';(root/'policy/splits').mkdir(parents=True);(root/'exclusions').mkdir()
    (root/'manifest.jsonl').write_text('immutable RGB/label manifest\n')
    (root/'policy-supervision.json').write_text(json.dumps({'policy_training_status':'blocked_pending_segment_review'}))
    (root/'exclusions/intervals.jsonl').write_text('')
    row=dict(recording='R',source_time_sec=2.,split='train',weak_training_eligible=True,
             policy_training_eligible=False,human_verified=False,
             proposed_training_target=dict(is_estimated=True,normalization=NORMALIZATION))
    split=root/'policy/splits/train-operator-intent.jsonl';split.write_text(json.dumps(row)+'\n')
    authorization=tmp_path/'authorization.json'
    def pin():
        authorization.write_text(json.dumps(dict(schema='r50_360.weak_training_authorization.v1',
            allow_weak_operator_intent_training=True,labels_are_verified=False,
            dataset_manifest_sha256=sha256(root/'manifest.jsonl'),
            policy_review_sha256=sha256(root/'policy-supervision.json'),
            exclusion_intervals_sha256=sha256(root/'exclusions/intervals.jsonl'),
            operator_split_sha256={'train':sha256(split)})))
    pin()
    with pytest.raises(ValueError,match='blocked pending'):
        read_rows(root,'train')
    accepted=read_rows(root,'train',authorization=authorization)
    assert accepted==[row] and accepted[0]['human_verified'] is False
    (root/'manifest.jsonl').write_text('different dataset\n')
    with pytest.raises(ValueError,match='does not match'):
        read_rows(root,'train',authorization=authorization)
    (root/'exclusions/intervals.jsonl').write_text(json.dumps(dict(recording='R',start_sec=1.,end_sec=3.))+'\n')
    pin()
    with pytest.raises(ValueError,match='excluded interval'):
        read_rows(root,'train',authorization=authorization)

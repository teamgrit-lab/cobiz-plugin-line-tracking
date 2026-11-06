"""Development-selected fine-tuning candidates with inference-matched speed caps."""
from collections import Counter
import copy
import json
import random

import numpy as np
import torch
from torch.nn import functional as F

from connectome_policy import DriveReadout
from r50_policy_package import POLICY_SCHEMA, sha256


def physical_action(action):
    """Differentiable version of the deployed curvature-preserving limits."""
    vx, yaw = action[:, 0]*10., action[:, 1]*.35
    factor = torch.minimum(torch.ones_like(vx), .8/vx.clamp_min(1e-8))
    factor = torch.minimum(factor, .18/yaw.abs().clamp_min(1e-8))
    return torch.stack([vx*factor, yaw*factor], dim=1)


def train_candidates(args, train, val, train_rows, val_rows, detector_pin):
    from finetune_r50_360_policy import evaluate, write_json, NORMALIZATION

    torch.set_num_threads(2)
    torch.manual_seed(args.seed)
    np.random.seed(args.seed)
    random.seed(args.seed)
    if sha256(args.parent_policy/'policy/drive-readout.pt') != args.parent_policy_sha256:
        raise ValueError('Parent actor hash mismatch')
    checkpoint = torch.load(args.parent_policy/'policy/drive-readout.pt', weights_only=True, map_location='cpu')
    if checkpoint.get('schema') != 'dangjin.drive_readout.vision.v2':
        raise ValueError('Pinned original vision-only actor required')
    parent = DriveReadout()
    parent.load_state_dict(checkpoint['state_dict'], strict=True)
    parent.eval().requires_grad_(False)
    tx, ty = train
    with torch.no_grad():
        ref_features = torch.cat([parent.vision(chunk) for chunk in tx.split(128)])
        ref_stop = torch.cat([parent.stop(chunk).softmax(1) for chunk in ref_features.split(128)])
    baseline, _ = evaluate(parent, *val, val_rows)
    write_json(args.run/'policy/baseline-validation.json', baseline)
    counts = Counter(r['action_group'] for r in train_rows)
    n, groups = len(train_rows), len(counts)
    probability = torch.tensor([.5/n + .5/(groups*counts[r['action_group']]) for r in train_rows])
    importance = (1./n)/probability
    candidates = [
        dict(name='action_only', lr=.0001, vision=False, stop=False),
        dict(name='heads', lr=.00005, vision=False, stop=True),
        dict(name='full_low_lr', lr=.00001, vision=True, stop=True),
    ]
    write_json(args.run/'policy/candidate-plan.json', dict(
        candidates=candidates, max_steps_each=args.steps,
        eval_every=args.eval_every, early_stopping_evaluations=5,
        stop_KL_weight=2., feature_distillation_weight=.05,
        stop_label_smoothing=.03,
        checkpoint_selection='development only; class-macro capped command errors + 0.5 stop CE',
        physical_caps=dict(vx_mps=.8, yaw_radps=.18), test_used_for_selection=False))
    global_best = float('inf')
    selected = None
    runs = []
    for candidate_index, config in enumerate(candidates):
        model = copy.deepcopy(parent).requires_grad_(True)
        if not config['vision']:
            model.vision.requires_grad_(False)
        if not config['stop']:
            model.stop.requires_grad_(False)
        optimizer = torch.optim.AdamW([p for p in model.parameters() if p.requires_grad],
                                      lr=config['lr'], weight_decay=.0001)
        generator = torch.Generator().manual_seed(args.seed+candidate_index)
        best, stale, best_step = float('inf'), 0, 0
        directory = args.run/'policy/candidates'/config['name']
        directory.mkdir(parents=True, exist_ok=True)
        for step in range(1, args.steps+1):
            model.train()
            indices = torch.multinomial(probability, args.batch_size, replacement=True, generator=generator)
            features = model.vision(tx[indices])
            raw = model.action(features)
            action = torch.stack([raw[:,0].sigmoid(), raw[:,1].tanh()], dim=1)
            logits = model.stop(features)
            target = ty[indices]
            moving = target[:,2]==0
            command = physical_action(action)
            regression = command.sum()*0.
            if moving.any():
                regression = (F.smooth_l1_loss(command[moving,0]/.8, target[moving,0]*10./.8, beta=.25)
                              + F.smooth_l1_loss(command[moving,1]/.18, target[moving,1]*.35/.18, beta=.1))
            ce = (F.cross_entropy(logits, target[:,2].long(), reduction='none', label_smoothing=.03)
                  *importance[indices]).mean()
            kl = F.kl_div(logits.log_softmax(1), ref_stop[indices], reduction='batchmean')
            distill = F.mse_loss(features, ref_features[indices])
            loss = regression + ce + 2.*kl + .05*distill
            if not torch.isfinite(loss):
                raise FloatingPointError('Nonfinite policy loss')
            optimizer.zero_grad(set_to_none=True)
            loss.backward()
            torch.nn.utils.clip_grad_norm_([p for p in model.parameters() if p.requires_grad], 1., error_if_nonfinite=True)
            optimizer.step()
            if step % args.eval_every and step != args.steps:
                continue
            metrics, _ = evaluate(model, *val, val_rows)
            record = dict(candidate=config['name'], step=step, loss=float(loss.detach()),
                          regression=float(regression.detach()), stop_ce=float(ce.detach()),
                          stop_KL=float(kl.detach()), validation=metrics)
            with (directory/'history.jsonl').open('a') as stream:
                stream.write(json.dumps(record, allow_nan=False)+'\n')
            print(json.dumps(dict(event='policy_validation', candidate=config['name'], step=step,
                                 score=metrics['checkpoint_score'], speed_mae=metrics['speed_mae_mps'],
                                 yaw_mae=metrics['yaw_mae_radps'], false_stop=metrics['false_stop'],
                                 missed_stop=metrics['missed_stop'])), flush=True)
            if metrics['checkpoint_score'] < best:
                best, best_step, stale = metrics['checkpoint_score'], step, 0
                payload = dict(schema=POLICY_SCHEMA, state_dict=model.state_dict(),
                               detector_manifest_sha256=detector_pin, action_normalization=NORMALIZATION,
                               step=step, candidate=config['name'])
                torch.save(payload, directory/'drive-readout.pt')
                if best < global_best:
                    global_best = best
                    selected = dict(candidate=config['name'], step=step, config=config,
                                    validation=metrics)
                    torch.save(payload, args.run/'policy/drive-readout.pt')
            else:
                stale += 1
            if stale >= 5:
                break
        runs.append(dict(config=config, optimizer_steps=step, selected_step=best_step,
                         best_validation_score=best, early_stopped=step<args.steps))
    final = DriveReadout()
    final.load_state_dict(torch.load(args.run/'policy/drive-readout.pt', weights_only=True,
                                    map_location='cpu')['state_dict'], strict=True)
    final.eval()
    report = dict(schema='r50_360.physical_intent_finetuning.v2',
        parent_policy_sha256=args.parent_policy_sha256,
        parent_policy_package_model_sha256=sha256(args.parent_policy/'model.json'),
        trained_detector_manifest_sha256=detector_pin,
        selected_step=selected['step'], selected_candidate=selected['candidate'],
        optimizer_steps=sum(r['optimizer_steps'] for r in runs), candidates=runs,
        selected_stop_head_finetuned=selected['config']['stop'],
        selected_visual_trunk_finetuned=selected['config']['vision'],
        seed=args.seed, batch_size=args.batch_size, inference_sample_rate_hz=2.5,
        temporal_state=False, actor_architecture_unchanged=True, flyvis_encoder_used=False,
        action_normalization=NORMALIZATION,
        physical_training_caps=dict(vx_mps=.8, yaw_radps=.18),
        training_group_counts=dict(counts),
        supervision='explicitly authorized weak recorded operator intent; not reviewed ideal or executed commands',
        training_authorization_sha256=sha256(args.training_authorization),
        ideal_commands_available=False, executed_velocity_available=False,
        train_split_sha256=sha256(args.dataset/'policy/splits/train-operator-intent.jsonl'),
        val_split_sha256=sha256(args.dataset/'policy/splits/val-operator-intent.jsonl'),
        exclusions_sha256=sha256(args.dataset/'exclusions/intervals.jsonl'),
        sampling='50% natural + 50% equal action groups; stop CE importance corrected',
        objective='curvature-preserving capped physical-space Huber speed/yaw; smoothed stop CE; parent stop KL; feature distillation',
        checkpoint_selection='development-only macro command errors + stop CE; test/external never fitted',
        baseline_validation=baseline, selected_validation=selected['validation'],
        validation_improved=global_best<baseline['checkpoint_score'],
        real_robot_validated=False, closed_loop_validated=False)
    write_json(args.run/'policy/training.json', report)
    return final, report

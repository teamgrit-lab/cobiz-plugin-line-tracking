#!/usr/bin/env python3
"""Fine-tune the existing CNN actor on explicitly opted-in operator intent.

The frozen R50 selected using development partial labels supplies every input.
Neither model proposals nor recorded joystick intent are called ideal labels.
Test labels are opened only after checkpoint selection. No robot is commanded.
"""
import argparse
from collections import Counter
import json
from pathlib import Path
import random
import time

import cv2
import numpy as np
import torch
from torch.nn import functional as F

from connectome_policy import DriveReadout, policy_pixels
from r50_360_policy_runtime import R50RoadPerception
from r50_checkpoint import checkpoint_manifest_sha256, validate_checkpoint
from r50_policy_package import POLICY_SCHEMA, SCHEMA, sha256
from surround_camera import load_profile, profile_sha256

NORMALIZATION = dict(vx_mps=10., yaw_radps=.35)


def write_json(path, value):
    Path(path).write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False) + '\n')


def validate_training_authorization(dataset, split, path):
    """Authorization permits weak training; it never verifies the target labels."""
    if path is None:
        return False
    value = json.loads(Path(path).read_text())
    if (value.get('schema') != 'r50_360.weak_training_authorization.v1'
            or value.get('allow_weak_operator_intent_training') is not True
            or value.get('labels_are_verified') is not False
            or value.get('dataset_manifest_sha256') != sha256(dataset/'manifest.jsonl')
            or value.get('policy_review_sha256') != sha256(dataset/'policy-supervision.json')
            or value.get('exclusion_intervals_sha256') != sha256(dataset/'exclusions/intervals.jsonl')
            or value.get('operator_split_sha256', {}).get(split) != sha256(
                dataset/'policy/splits'/f'{split}-operator-intent.jsonl')):
        raise ValueError('Weak training authorization does not match this dataset/split')
    return True


def read_rows(dataset, split, *, authorization=None):
    permitted = validate_training_authorization(dataset, split, authorization)
    review_paths = [dataset / 'policy-supervision.json',
                    dataset.with_name(dataset.name + '.review') / 'policy-supervision.json']
    for review_path in review_paths:
        if not review_path.exists():
            continue
        review = json.loads(review_path.read_text())
        if review.get('policy_training_status') != 'approved_curated_dataset' and not permitted:
            raise ValueError(
                'Source contains mixed normal/failed/intervened driving. '
                'Operator-intent training is blocked pending segment review: ' + str(review_path))
    rows = [json.loads(v) for v in (dataset / 'policy/splits' / f'{split}-operator-intent.jsonl').read_text().splitlines() if v.strip()]
    for row in rows:
        if (row['split'] != split or not row['weak_training_eligible']
                or row['policy_training_eligible'] or row['human_verified']
                or not row['proposed_training_target']['is_estimated']
                or row['proposed_training_target']['normalization'] != NORMALIZATION):
            raise ValueError('Expected separate, explicitly weak operator-intent targets')
    # Even explicitly authorized weak training must obey the curated exclusions.
    exclusions = dataset/'exclusions/intervals.jsonl'
    if exclusions.exists():
        intervals = [json.loads(v) for v in exclusions.read_text().splitlines() if v.strip()]
        if any(r['recording']==e['recording'] and e['start_sec']<=r['source_time_sec']<e['end_sec']
               for r in rows for e in intervals):
            raise ValueError('An excluded interval entered operator-intent supervision')
    return rows


def cache_inputs(dataset, run, rows, split, detector, detector_pin):
    directory = run / 'feature-cache' / split
    directory.mkdir(parents=True, exist_ok=True)
    contract = dict(detector_manifest_sha256=detector_pin,
                    split_manifest_sha256=sha256(dataset / 'policy/splits' / f'{split}-operator-intent.jsonl'),
                    schema='r50_360.policy_features.v1', actor_input_chw=[4, 32, 64],
                    actor_crop_rows=[135, 257], road_source='new frozen R50 predictions',
                    inference_precision=detector.details['precision'])
    contract_path = directory / 'contract.json'
    if contract_path.exists() and json.loads(contract_path.read_text()) != contract:
        raise ValueError('Feature cache contract changed')
    write_json(contract_path, contract)
    chunks, start = [], time.perf_counter()
    for offset in range(0, len(rows), 64):
        selected = rows[offset:offset + 64]
        path = directory / f'{offset:06d}.npz'
        ids = np.asarray([r['id'] for r in selected])
        if path.exists():
            with np.load(path, allow_pickle=False) as cached:
                if not np.array_equal(cached['ids'], ids):
                    raise ValueError('Feature IDs changed')
                chunks.append(cached['x'].copy())
        else:
            features = []
            for row in selected:
                views = []
                for view in row['views']:
                    image_path = (dataset / view['image']).resolve()
                    if not image_path.is_relative_to(dataset) or sha256(image_path) != view['sha256']:
                        raise ValueError('Camera image path/hash mismatch')
                    image = cv2.imread(str(image_path))
                    if image is None or image.shape != (270, 480, 3):
                        raise ValueError('Invalid camera view')
                    views.append(image)
                masks, _ = detector.predict_views(views)
                features.append(policy_pixels(masks))
            x = np.asarray(features, np.float16)
            with path.with_suffix('.tmp').open('wb') as handle:
                np.savez_compressed(handle, x=x, ids=ids)
            path.with_suffix('.tmp').replace(path)
            chunks.append(x)
        print(json.dumps(dict(event='cache', split=split, done=min(offset + 64, len(rows)),
                              total=len(rows), elapsed_sec=round(time.perf_counter()-start, 1))), flush=True)
    x = np.concatenate(chunks).astype(np.float32)
    y = np.asarray([r['proposed_training_target']['normalized_action'] for r in rows], np.float32)
    if x.shape != (len(rows), 4, 32, 64) or not np.isfinite(x).all():
        raise ValueError('Invalid policy input cache')
    return torch.from_numpy(x), torch.from_numpy(y)


def evaluate(model, x, y, rows):
    model.eval()
    with torch.no_grad():
        predictions = [model(chunk) for chunk in x.split(128)]
        action = torch.cat([p[0] for p in predictions])
        logits = torch.cat([p[1] for p in predictions])
        p_stop = logits.softmax(1)[:, 1]
        stopped = p_stop > .5
        target_stop = y[:, 2].bool()
        v, yaw = action[:, 0] * 10., action[:, 1] * .35
        # Actual actor commands, before the unchanged physical-cap adapter.
        v, yaw = v.masked_fill(stopped, 0.), yaw.masked_fill(stopped, 0.)
        factor = torch.minimum(torch.ones_like(v), .8 / v.clamp_min(1e-8))
        factor = torch.minimum(factor, .18 / yaw.abs().clamp_min(1e-8))
        v, yaw = v * factor, yaw * factor
        error_v = (v - y[:, 0] * 10.).abs()
        error_yaw = (yaw - y[:, 1] * .35).abs()
        ce = F.cross_entropy(logits, target_stop.long(), reduction='none')
        groups = {}
        for group in sorted({r['action_group'] for r in rows}):
            indices = torch.tensor([i for i, r in enumerate(rows) if r['action_group'] == group])
            groups[group] = dict(samples=len(indices), speed_mae_mps=float(error_v[indices].mean()),
                                 yaw_mae_radps=float(error_yaw[indices].mean()),
                                 stop_cross_entropy=float(ce[indices].mean()))
        # Development-only, class-macro score prevents straight-only domination.
        score = np.mean([g['speed_mae_mps']/.8 + g['yaw_mae_radps']/.18
                         + .5*g['stop_cross_entropy'] for g in groups.values()])
        tp = int((stopped & target_stop).sum())
        fp = int((stopped & ~target_stop).sum())
        fn = int((~stopped & target_stop).sum())
        report = dict(samples=len(rows), checkpoint_score=float(score),
                      speed_mae_mps=float(error_v.mean()), yaw_mae_radps=float(error_yaw.mean()),
                      stop_true_positive=tp, false_stop=fp, missed_stop=fn,
                      target_stop_count=int(target_stop.sum()), groups=groups,
                      reference='unreviewed recorded operator intent, NOT ideal or measured execution',
                      physical_caps=dict(vx_mps=.8, yaw_radps=.18), closed_loop_validated=False)
        return report, dict(vx_mps=v.numpy(), yaw_radps=yaw.numpy(),
                            stop_probability=p_stop.numpy(), stopped=stopped.numpy())


def train_actor(args, train, val, train_rows, val_rows, detector_pin):
    torch.set_num_threads(2)
    torch.manual_seed(args.seed)
    random.seed(args.seed)
    np.random.seed(args.seed)
    if sha256(args.parent_policy / 'policy/drive-readout.pt') != args.parent_policy_sha256:
        raise ValueError('Parent policy SHA-256 mismatch')
    parent = torch.load(args.parent_policy / 'policy/drive-readout.pt', map_location='cpu', weights_only=True)
    if parent.get('schema') != 'dangjin.drive_readout.vision.v2':
        raise ValueError('Expected existing vision-only CNN policy')
    model = DriveReadout()
    model.load_state_dict(parent['state_dict'], strict=True)
    reference = {k: v.clone() for k, v in model.state_dict().items()}
    baseline, _ = evaluate(model, *val, val_rows)
    write_json(args.run / 'policy/baseline-validation.json', baseline)
    optimizer = torch.optim.AdamW(model.parameters(), lr=args.lr, weight_decay=.0001)
    counts = Counter(r['action_group'] for r in train_rows)
    n, group_count = len(train_rows), len(counts)
    probability = torch.tensor([.5/n + .5/(group_count*counts[r['action_group']]) for r in train_rows])
    # Correct stop CE back to natural prevalence despite turn-balanced sampling.
    # Six training stops must not become an artificial 25% stop prior.
    stop_importance = (1./n)/probability
    generator = torch.Generator().manual_seed(args.seed)
    tx, ty = train
    best, best_step = float('inf'), 0
    history = args.run / 'policy/history.jsonl'
    for step in range(1, args.steps + 1):
        model.train()
        indices = torch.multinomial(probability, args.batch_size, replacement=True, generator=generator)
        action, stop_logits = model(tx[indices])
        target = ty[indices]
        moving = target[:, 2] == 0
        if moving.any():
            speed_loss = F.smooth_l1_loss(action[moving, 0]*10./.8, target[moving, 0]*10./.8, beta=.25)
            yaw_loss = F.smooth_l1_loss(action[moving, 1]*.35/.18, target[moving, 1]*.35/.18, beta=.1)
        else:
            speed_loss = yaw_loss = action.sum()*0.
        stop_loss = (F.cross_entropy(stop_logits, target[:, 2].long(), reduction='none')
                     * stop_importance[indices]).mean()
        retention = sum((p-reference[name]).square().mean() for name, p in model.named_parameters())
        loss = speed_loss + yaw_loss + stop_loss + .01*retention
        if not torch.isfinite(loss):
            raise FloatingPointError('Nonfinite policy loss')
        optimizer.zero_grad(set_to_none=True)
        loss.backward()
        torch.nn.utils.clip_grad_norm_(model.parameters(), 1., error_if_nonfinite=True)
        optimizer.step()
        if step % args.eval_every == 0 or step == args.steps:
            metrics, _ = evaluate(model, *val, val_rows)
            record = dict(step=step, loss=float(loss.detach()), validation=metrics)
            with history.open('a') as stream:
                stream.write(json.dumps(record, allow_nan=False)+'\n')
            print(json.dumps(dict(event='policy_validation', **record)), flush=True)
            if metrics['checkpoint_score'] < best:
                best, best_step = metrics['checkpoint_score'], step
                torch.save(dict(schema=POLICY_SCHEMA, state_dict=model.state_dict(),
                                detector_manifest_sha256=detector_pin,
                                action_normalization=NORMALIZATION, step=step),
                           args.run / 'policy/drive-readout.pt')
    checkpoint = torch.load(args.run / 'policy/drive-readout.pt', map_location='cpu', weights_only=True)
    model.load_state_dict(checkpoint['state_dict'], strict=True)
    report = dict(schema='r50_360.operator_intent_finetuning.v1',
                  parent_policy_sha256=args.parent_policy_sha256,
                  parent_policy_package_model_sha256=sha256(args.parent_policy/'model.json'),
                  trained_detector_manifest_sha256=detector_pin, selected_step=best_step,
                  optimizer_steps=args.steps, seed=args.seed, lr=args.lr,
                  batch_size=args.batch_size, inference_sample_rate_hz=2.5,
                  temporal_state=False, actor_architecture_unchanged=True,
                  flyvis_encoder_used=False, action_normalization=NORMALIZATION,
                  supervision='weak unreviewed recorded operator intent; explicitly opted in',
                  ideal_commands_available=False, executed_velocity_available=False,
                  training_group_counts=dict(counts),
                  train_split_sha256=sha256(args.dataset/'policy/splits/train-operator-intent.jsonl'),
                  val_split_sha256=sha256(args.dataset/'policy/splits/val-operator-intent.jsonl'),
                  sampling='50% natural + 50% equal action groups; stop CE importance corrected',
                  objective='metric-scale Huber speed/yaw + natural-prior stop CE + L2 retention',
                  checkpoint_selection='development class-macro command errors + stop CE',
                  baseline_validation=baseline, selected_validation=evaluate(model, *val, val_rows)[0],
                  real_robot_validated=False, closed_loop_validated=False)
    write_json(args.run / 'policy/training.json', report)
    return model, report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset', type=Path, required=True)
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--parent-policy', type=Path, required=True)
    parser.add_argument('--parent-policy-sha256', required=True)
    parser.add_argument('--allow-unreviewed-operator-intent', action='store_true')
    parser.add_argument('--training-authorization', type=Path,
                        help='Explicit user-request record pinned to this dataset; labels remain weak')
    parser.add_argument('--training-mode', choices=['legacy', 'physical-candidates'], default='legacy')
    parser.add_argument('--detector-device', default='mps', choices=['mps', 'cuda', 'cpu'])
    parser.add_argument('--steps', type=int, default=2400)
    parser.add_argument('--eval-every', type=int, default=100)
    parser.add_argument('--batch-size', type=int, default=64)
    parser.add_argument('--lr', type=float, default=.0003)
    parser.add_argument('--seed', type=int, default=1105)
    args = parser.parse_args()
    if not args.allow_unreviewed_operator_intent:
        parser.error('No verified ideal targets: explicit weak-supervision opt-in required')
    args.dataset, args.run = args.dataset.resolve(), args.run.resolve()
    (args.run/'policy').mkdir(exist_ok=True)
    if (args.run/'policy/drive-readout.pt').exists():
        raise ValueError('Refusing to overwrite an already trained policy')
    detector_directory = args.run/'detector/best'
    detector_pin = checkpoint_manifest_sha256(detector_directory)
    validate_checkpoint(detector_directory, detector_pin, require_pin=True)
    train_rows = read_rows(args.dataset, 'train', authorization=args.training_authorization)
    val_rows = read_rows(args.dataset, 'val', authorization=args.training_authorization)
    if {r['id'] for r in train_rows} & {r['id'] for r in val_rows}:
        raise ValueError('Train/development ID overlap')
    detector = R50RoadPerception(detector_directory, detector_pin, args.detector_device)
    train = cache_inputs(args.dataset, args.run, train_rows, 'train', detector, detector_pin)
    val = cache_inputs(args.dataset, args.run, val_rows, 'val', detector, detector_pin)
    if args.training_mode == 'physical-candidates':
        from r50_policy_finetuning import train_candidates
        model, report = train_candidates(args, train, val, train_rows, val_rows, detector_pin)
    else:
        model, report = train_actor(args, train, val, train_rows, val_rows, detector_pin)
    # First access to held-out policy targets; no subsequent fitting is allowed.
    test_rows = read_rows(args.dataset, 'test', authorization=args.training_authorization)
    if ({r['id'] for r in test_rows} & {r['id'] for r in train_rows+val_rows}):
        raise ValueError('Held-out policy ID overlap')
    test = cache_inputs(args.dataset, args.run, test_rows, 'test', detector, detector_pin)
    metrics, predictions = evaluate(model, *test, test_rows)
    write_json(args.run/'policy/test-evaluation.json', metrics)
    with (args.run/'policy/test-predictions.jsonl').open('w') as stream:
        for i, row in enumerate(test_rows):
            stream.write(json.dumps(dict(id=row['id'], source_time_sec=row['source_time_sec'],
                         vx_mps=float(predictions['vx_mps'][i]), yaw_radps=float(predictions['yaw_radps'][i]),
                         stop_probability=float(predictions['stop_probability'][i]),
                         reference_is_estimated=True, ideal_command=None))+'\n')
    config = dict(schema=SCHEMA, detector_directory='detector/best',
                  detector_manifest_sha256=detector_pin, policy_trained_detector_sha256=detector_pin,
                  policy_sha256=sha256(args.run/'policy/drive-readout.pt'),
                  policy_training_sha256=sha256(args.run/'policy/training.json'),
                  action_normalization=NORMALIZATION, physical_defaults=dict(vx_mps=.8, yaw_radps=.18),
                  camera_profile_sha256=profile_sha256(load_profile()),
                  input_contract=dict(panorama_shape_wh=[2880,1440], camera_views=['front','left','rear','right'],
                      view_shape_wh=[480,270], forward_heading_deg=90, actor_input_chw=[4,32,64], actor_crop_rows=[135,257]),
                  dataset_manifest_sha256=sha256(args.dataset/'manifest.jsonl'),
                  real_robot_validated=False, closed_loop_validated=False,
                  supervision='assistant partial segmentation labels + weak operator intent',
                  simulation_only=True, runtime_planner=False, flyvis_encoder_used=False)
    write_json(args.run/'model.json', config)
    print(json.dumps(dict(event='complete', model_json_sha256=sha256(args.run/'model.json'),
                          policy_sha256=config['policy_sha256'], detector_sha256=detector_pin,
                          test=metrics)), flush=True)


if __name__ == '__main__':
    main()

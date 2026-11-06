#!/usr/bin/env python3
"""Fit STOP ablations on reviewed R50 crops, keeping detector/actions frozen.

Input directory must contain reviewed-decisions.json, source-observations.jsonl
and frozen-r50-features.npz. Assistant preferences and constructed blockage
labels are explicitly distinct; original recorded pauses are not STOP truth.
"""
import argparse
from collections import Counter
import json
from pathlib import Path
import shutil

import cv2
import numpy as np
import torch
from torch.nn import functional as F

from connectome_policy import DriveReadout
from r50_front_stop_policy import (ARCHITECTURES, FrontStopReadout, OBSERVATION_SPEC,
                                   PAIR_SCHEMA, POLICY_SCHEMA, TIME_SPEC)
from r50_policy_package import sha256


def save(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False) + '\n')


def synthetic(count, seed, real_features=None):
    """Known constructed mask layouts, never claimed to be real obstacle truth."""
    rng = np.random.default_rng(seed)
    x = np.zeros((count, 5, 4, 32, 64), np.float32)
    target = np.arange(count) % 2
    dt = rng.uniform(.28, .55, (count, 5)).astype(np.float32)
    dt[:, 0] = 0
    yy, xx = np.mgrid[:32, :64]
    for i in range(count):
        center = rng.uniform(20, 44)
        curve = rng.uniform(-10, 10)
        width = rng.uniform(20, 57)
        transition = rng.integers(1, 6)
        for k in range(5):
            c = center + curve * (1 - yy / 31) ** 2 + rng.normal(0, .6)
            corridor = abs(xx - c) < width * (.55 + .45 * yy / 31) / 2
            # STOP layouts have an observed full-width break in the near road,
            # sometimes with far road and isolated false positive speckles.
            blocked = (k >= 5 - transition) if target[i] else (k < 5 - transition and i % 4 == 0)
            if blocked:
                corridor[yy >= rng.integers(6, 18)] = False
                for _ in range(rng.integers(0, 4)):
                    y, z = rng.integers(19, 31), rng.integers(0, 63)
                    corridor[y:y + 1, z:z + 2] = True
            else:
                # Moderate transient segmentation holes, not a privileged
                # detector-failure flag or an instruction to drive when blind.
                for _ in range(rng.integers(0, 5)):
                    y, z = rng.integers(0, 26), rng.integers(0, 59)
                    corridor[y:y + rng.integers(1, 5), z:z + rng.integers(1, 6)] = False
            x[i, k, 0] = corridor
            for direction in (1, 2, 3):
                if rng.random() < .45:
                    continue
                side_center = rng.uniform(0, 64)
                x[i, k, direction] = (abs(xx - side_center) < rng.uniform(5, 55)) & (yy > rng.integers(0, 18))
            # Match fractional INTER_AREA edge values of real detector crops;
            # otherwise a classifier can separate crisp synthetic masks from
            # camera masks instead of learning passage versus blockage.
            for direction in range(4):
                native = cv2.resize(x[i, k, direction], (480, 122), interpolation=cv2.INTER_NEAREST)
                x[i, k, direction] = cv2.resize(native, (64, 32), interpolation=cv2.INTER_AREA)
        if real_features is not None and i % 4 < 2:
            # Explicit counterfactual mask edits, not real STOP annotations.
            # GO and blocked variants share camera texture/edge statistics.
            base = real_features[int(rng.integers(len(real_features)))].copy()
            x[i] = base
            if target[i]:
                for k in range(5 - transition, 5):
                    x[i, k, 0, int(rng.integers(5, 16)):] = 0
        if target[i] and i % 8 == 1:
            # No Road evidence and tiny isolated false positives are explicitly
            # negative. Never label a completely blank mask as GO.
            for k in range(5 - transition, 5):
                x[i, k, 0] = 0
                if i % 16 == 1:
                    x[i, k, 0, rng.integers(0, 30), rng.integers(0, 62)] = rng.random()
    return x, target.astype(np.int64), dt


def real_sequences(root):
    reviews = json.loads((root / 'reviewed-decisions.json').read_text())
    rows = [json.loads(s) for s in (root / 'source-observations.jsonl').read_text().splitlines()]
    with np.load(root / 'frozen-r50-features.npz', allow_pickle=False) as z:
        features, ids = z['x'], z['ids']
    assert len(features) == len(rows) and all(r['id'] == sid for r, sid in zip(rows, ids))
    result = {}
    for split in ('train', 'val'):
        xs, ys, ds, kept = [], [], [], []
        for review in reviews:
            if (not review['reviewed'] or review.get('policy_training_eligible') is False or review['split'] != split
                    or review['decision_label'] not in (0, 1)):
                continue
            index = review['feature_index']
            end = rows[index]
            assert not end.get('excluded_from_training') and end['split'] == split
            indices = [index]
            while len(indices) < 5 and indices[0] > 0:
                a, b = rows[indices[0] - 1], rows[indices[0]]
                if (a['recording'] != b['recording'] or a['split'] != split
                        or a.get('excluded_from_training')
                        or b['time_bin'] - a['time_bin'] != 1
                        or not 0 < b['source_time_sec'] - a['source_time_sec'] <= TIME_SPEC['max_gap_sec']):
                    break
                indices.insert(0, indices[0] - 1)
            # Left padding is the same first capture, with zero elapsed time;
            # mark it in the manifest. Cold starts are fitted separately too.
            padded = [indices[0]] * (5 - len(indices)) + indices
            dt = np.diff([rows[j]['source_time_sec'] for j in padded], prepend=rows[padded[0]]['source_time_sec'])
            xs.append(features[padded]); ys.append(review['decision_label']); ds.append(dt)
            kept.append(dict(id=end['id'], source_ids=[rows[j]['id'] for j in indices],
                             split=split, target=review['decision_label'], padding_captures=5 - len(indices)))
        result[split] = (np.stack(xs), np.asarray(ys, np.int64), np.asarray(ds, np.float32), kept)
    save(root / 'reviewed-sequences.json', {split: values[3] for split, values in result.items()})
    return result


@torch.inference_mode()
def evaluate(model, x, y, dt):
    probs = []
    for begin in range(0, len(x), 64):
        _, logits = model(torch.from_numpy(x[begin:begin + 64]), torch.from_numpy(dt[begin:begin + 64]))
        probs.extend(logits.softmax(1)[:, 1].tolist())
    p = np.asarray(probs)
    stop = p > .5
    return dict(count=len(y), stop=int(stop.sum()),
                false_stop=int(((y == 0) & stop).sum()),
                missed_stop=int(((y == 1) & ~stop).sum()),
                stop_recall=float(stop[y == 1].mean()) if (y == 1).any() else None,
                go_false_stop_rate=float(stop[y == 0].mean()) if (y == 0).any() else None,
                nll=float(-(np.log(np.clip(np.where(y, p, 1 - p), 1e-7, 1))).mean()))


def train(args):
    torch.set_num_threads(2)
    root, output, parent = Path(args.dataset), Path(args.output), Path(args.parent)
    config = json.loads((parent / 'model.json').read_text())
    original = DriveReadout()
    original.load_state_dict(torch.load(parent / 'policy/drive-readout.pt', map_location='cpu', weights_only=True)['state_dict'])
    original.eval().requires_grad_(False)
    real = real_sequences(root)
    syn = {}
    for split, count, seed in [('train', 1600, 110601), ('val', 512, 110602), ('test', 512, 110603)]:
        syn[split] = synthetic(count, seed, real[split][0] if split in real else None)
        np.savez_compressed(root / f'synthetic-known-blockage-{split}.npz',
                            x=syn[split][0], y=syn[split][1], dt=syn[split][2])
    save(root / 'supervision.json', dict(
        real_review_labels={s: dict(Counter(str(y) for y in real[s][1])) for s in real},
        real_required_stop_examples=0, real_labels='assistant qualitative visual preference, not human ground truth',
        synthetic={s: dict(count=len(v[1]), seed=110601 + i, classes=dict(Counter(str(y) for y in v[1]))) for i, (s, v) in enumerate(syn.items())},
        uncertain='255 excluded', parent_stop_KL_weight=0., steering_retrained=False,
        detector_frozen=True, actions_frozen=True, stop_threshold=.5,
        actual_camera_timing='source PTS, max gap .65 sec; nominal .4 is dt scaling, not an inference throttle',
        no_external_or_test_fitting=True, observation_spec=OBSERVATION_SPEC, time_spec=TIME_SPEC))
    report = {}
    for architecture in ARCHITECTURES:
        torch.manual_seed(1106)
        rng = np.random.default_rng(1106)
        model = FrontStopReadout(architecture)
        model.retain_action(original)
        optimizer = torch.optim.AdamW([p for p in model.parameters() if p.requires_grad], lr=8e-4, weight_decay=1e-4)
        log, best, best_state = [], None, None
        for step in range(1, args.steps + 1):
            # Equal real-GO and constructed-layout losses. No inherited STOP KL,
            # no weak joystick STOP, no test observation contributes gradients.
            ri = rng.integers(len(real['train'][1]), size=16)
            si = rng.integers(len(syn['train'][1]), size=32)
            x = np.concatenate([real['train'][0][ri], syn['train'][0][si]])
            y = np.concatenate([real['train'][1][ri], syn['train'][1][si]])
            dt = np.concatenate([real['train'][2][ri], syn['train'][2][si]])
            # Train cold starts and shorter contexts instead of requiring a
            # history to authorize the first live decision.
            length = int(rng.integers(1, 6)) if architecture == 'temporal_front' else 1
            used_dt = dt[:, -length:].copy()
            used_dt[:, 0] = 0.
            _, logits = model(torch.from_numpy(x[:, -length:]), torch.from_numpy(used_dt))
            losses = F.cross_entropy(logits, torch.from_numpy(y), reduction='none')
            loss = .5 * losses[:16].mean() + .5 * losses[16:].mean()
            optimizer.zero_grad(); loss.backward()
            torch.nn.utils.clip_grad_norm_([p for p in model.parameters() if p.requires_grad], 5.)
            optimizer.step()
            if step % 100 == 0 or step == args.steps:
                rv = evaluate(model, *real['val'][:3])
                sv = evaluate(model, *syn['val'])
                # Do not select a trivial always-GO readout. Development only.
                score = rv['go_false_stop_rate'] + sv['go_false_stop_rate'] + 10 * (1 - sv['stop_recall'])
                row = dict(step=step, loss=float(loss.detach()), real_val=rv, synthetic_val=sv, selection_score=score)
                log.append(row)
                if best is None or (score, sv['nll'] + rv['nll']) < (best['selection_score'], best['synthetic_val']['nll'] + best['real_val']['nll']):
                    best = row
                    best_state = {k: v.detach().clone() for k, v in model.state_dict().items()}
                print(json.dumps(dict(architecture=architecture, **row)), flush=True)
        model.load_state_dict(best_state)
        training = dict(architecture=architecture, selected=best, updates=args.steps,
            parent_stop_KL_weight=0., detector_frozen=True, steering_retrained=False,
            original_action_weights_sha256=config['policy_sha256'],
            reviewed_decisions_sha256=sha256(root / 'reviewed-decisions.json'),
            source_features_sha256=sha256(root / 'frozen-r50-features.npz'),
            observation_spec=OBSERVATION_SPEC, time_spec=TIME_SPEC,
            limitations=['No verified real full-width blockage STOP ground truth in the reviewed set.',
                        'Constructed blockage recall is not real robot collision validation.',
                        'Velocity/yaw branch is unchanged; recorded-video replay is open loop.'])
        package = output / architecture / 'package'
        (package / 'policy').mkdir(parents=True, exist_ok=True)
        torch.save(dict(schema=POLICY_SCHEMA, architecture=architecture,
                        detector_manifest_sha256=config['detector_manifest_sha256'],
                        action_normalization=config['action_normalization'],
                        observation_spec=OBSERVATION_SPEC, time_spec=TIME_SPEC,
                        state_dict=best_state), package / 'policy/drive-readout.pt')
        save(package / 'policy/training.json', training)
        # A candidate package remains portable: the exact frozen detector is
        # copied, not retrained or replaced with a differently pinned version.
        shutil.copytree(parent / 'detector/best', package / 'detector/best',
                        dirs_exist_ok=True, ignore=shutil.ignore_patterns('._*'))
        paired = {k: v for k, v in config.items() if k not in ('schema', 'policy_sha256', 'policy_training_sha256')}
        paired.update(schema=PAIR_SCHEMA, policy_schema=POLICY_SCHEMA,
            architecture=architecture, stop_input_chw=[4 if architecture == 'reset_shared' else 2, 32, 64],
            stop_uses_side_views=architecture == 'reset_shared',
            observation_spec=OBSERVATION_SPEC, time_spec=TIME_SPEC,
            policy_sha256=sha256(package / 'policy/drive-readout.pt'),
            policy_training_sha256=sha256(package / 'policy/training.json'),
            real_robot_validated=False, simulation_only=False,
            parent_package_sha256=sha256(parent / 'model.json'))
        save(package / 'model.json', paired)
        save(output / architecture / 'fit-log.json', log)
        report[architecture] = dict(selected=best, package=str(package), model_sha256=sha256(package / 'model.json'))
    # Select architecture using the same development criterion, not replay/test.
    selected = min(report, key=lambda k: (report[k]['selected']['selection_score'],
        report[k]['selected']['real_val']['nll'] + report[k]['selected']['synthetic_val']['nll']))
    # A side-dependent diagnostic may not become the deployment recommendation.
    selected_front = min(('front', 'temporal_front'), key=lambda k: (
        report[k]['selected']['selection_score'], report[k]['selected']['real_val']['nll'] + report[k]['selected']['synthetic_val']['nll']))
    save(output / 'training-report.json', dict(candidates=report, best_dev=selected,
        selected_front_candidate=selected_front, selection_uses='development only',
        real_required_stop_validation='pending', detector_sha256=config['detector_manifest_sha256']))
    print(json.dumps(dict(event='training_complete', selected=selected_front, output=str(output))), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset', required=True)
    parser.add_argument('--parent', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--steps', type=int, default=1000)
    train(parser.parse_args())

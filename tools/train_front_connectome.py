#!/usr/bin/env python3
"""Train front-only readouts on real pseudo labels and offline simulator images."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import shutil
import time

import cv2
import numpy as np
import torch
from torch.nn import functional as F

from front_connectome import SCHEMA, FrontDriveReadout, front_pixels, sha256
from connectome_flow import FlyvisFlow
from connectome_road import CircuitFeatures, RoadReadout
from dangjin_sensor_faults import SensorFaults
from train_connectome_policy import teacher

ROOT = Path(__file__).resolve().parents[1]
BUNDLE_SHA = '97f853183f79d2e987532ca2716ce3c33c2f41f875af05396e0cd8b14a7bc3c2'


def save_json(path, data):
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False) + '\n')


def entries(root, exclusions=None):
    samples = {r['id']: r for r in map(json.loads, (root / 'samples.jsonl').read_text().splitlines())}
    assignment = json.loads((root / 'splits/policy.json').read_text())['assignments']
    result = []
    for label in map(json.loads, (root / 'pseudo_labels.jsonl').read_text().splitlines()):
        split = assignment[label['session']]
        if split == 'review':
            continue
        result.append({'id': label['id'], 'image': str(root / samples[label['id']]['camera']['image']),
            'source': 'real_pseudo', 'split': split, 'label': label['labels']['road'],
            'command': label['estimated_command']})
    folder = ROOT / '.cache/dangjin-connectome-training'
    for p in sorted(folder.glob('[0-9][0-9][0-9].json')):
        m = json.loads(p.read_text())
        if m['sector'] != 0:
            continue
        result.append({'id': 'sim-road-' + p.stem, 'source': 'simulation_road',
            'split': 'val' if m['pose_group'] % 5 == 0 else 'train',
            'image': str(p.with_name(p.stem + '-rgb.png')),
            'mask_image': str(p.with_name(p.stem + '-road.png')), 'command': None})
    route = np.asarray(json.loads((ROOT / 'environments/dangjin/world.json').read_text())['perimeter_route'])
    folder = ROOT / '.cache/dangjin-policy-training'
    for p in sorted(folder.glob('[0-9][0-9][0-9][0-9].json')):
        m = json.loads(p.read_text()); action = teacher(m, route)
        result.append({'id': 'sim-policy-' + p.stem, 'source': 'simulation_policy',
            'split': 'val' if m['pose_group'] % 5 == 0 else 'train',
            'image': str(p.with_name(p.stem + '-0-rgb.png')),
            'command': {'linear_x_mps': float(action[0] * .8), 'angular_z_radps': float(action[1] * .35),
                        'stop': False, 'is_estimated': True, 'teacher': 'offline simulation geometry'}})
    if exclusions:
        excluded = set(json.loads(Path(exclusions).read_text())['excluded_ids'])
        result = [r for r in result if r['id'] not in excluded]
    return result


def build_features(args, rows):
    cache = args.cache; cache.mkdir(parents=True, exist_ok=True)
    identity = hashlib.sha256(json.dumps(rows, sort_keys=True).encode()).hexdigest()
    info_path = cache / 'features-progress.json'
    old = json.loads(info_path.read_text()) if info_path.exists() else None
    if old and old['index_sha256'] != identity:
        raise ValueError('Feature cache belongs to different samples')
    done = old['completed'] if old else 0
    shape = (len(rows), 34, 48, 88)
    values = np.lib.format.open_memmap(cache / 'features.npy', mode='r+' if old else 'w+', dtype=np.float16, shape=shape)
    labels = np.lib.format.open_memmap(cache / 'targets.npy', mode='r+' if old else 'w+', dtype=np.uint8, shape=(len(rows), 96, 176))
    encoder = FlyvisFlow(args.bundle, BUNDLE_SHA, 'cpu'); extract = CircuitFeatures(encoder)
    shard_name = None; shard = None
    for start in range(done, len(rows), 8):
        batch = rows[start:start + 8]
        images = [cv2.resize(cv2.imread(r['image']), (480, 270), interpolation=cv2.INTER_AREA) for r in batch]
        values[start:start + len(batch)] = extract(images).astype(np.float16)
        for j, row in enumerate(batch):
            if 'label' in row:
                ref = row['label']
                if shard_name != ref['archive']:
                    with np.load(args.dataset / ref['archive'], allow_pickle=False) as z:
                        shard = z['road']
                    shard_name = ref['archive']
                target = shard[ref['index']]
            elif 'mask_image' in row:
                target = cv2.imread(row['mask_image'], 0)
            else:
                target = np.full((96, 176), 255, np.uint8)
            labels[start + j] = cv2.resize(target, (176, 96), interpolation=cv2.INTER_NEAREST)
        values.flush(); labels.flush()
        save_json(info_path, {'index_sha256': identity, 'completed': start + len(batch),
                             'total': len(rows), 'encoder_sha256': BUNDLE_SHA})
        if start % 64 == 0:
            print(json.dumps({'stage': 'features', 'completed': start + len(batch), 'total': len(rows)}), flush=True)
    save_json(cache / 'entries.json', rows)
    del encoder, extract, values, labels
    torch.set_default_device('cpu')


def road_scores(model, x, y, ids, device, threshold=.5):
    tp = fp = fn = 0
    model.eval()
    with torch.inference_mode():
        for start in range(0, len(ids), 16):
            batch = ids[start:start + 16]
            inp = torch.as_tensor(x[batch].astype(np.float32), device=device)
            target = torch.as_tensor(y[batch].copy(), device=device)[:, None]
            p = F.interpolate(model(inp), size=(96, 176), mode='bilinear', align_corners=False).sigmoid() >= threshold
            valid = target != 255; truth = target == 1
            tp += int((p & truth & valid).sum()); fp += int((p & ~truth & valid).sum()); fn += int((~p & truth & valid).sum())
    return {'iou': tp / max(1, tp + fp + fn), 'precision': tp / max(1, tp + fp), 'recall': tp / max(1, tp + fn)}


def train_road(args, rows, x, y, device):
    model = RoadReadout().to(device)
    initial = ROOT / '.cache/dangjin-connectome-road/road-readout.pt'
    model.load_state_dict(torch.load(initial, map_location='cpu', weights_only=True)['state_dict'])
    train = np.array([i for i, r in enumerate(rows) if r['split'] == 'train' and r['source'] != 'simulation_policy'])
    sim = np.array([i for i in train if rows[i]['source'] == 'simulation_road'])
    valid = np.array([i for i, r in enumerate(rows) if r['split'] == 'val' and r['source'] == 'real_pseudo'])
    # Keep the pretrained activation normalization; fine-tune all readout layers.
    opt = torch.optim.AdamW(model.parameters(), lr=.00025, weight_decay=.0001)
    schedule = torch.optim.lr_scheduler.CosineAnnealingLR(opt, args.road_epochs, eta_min=.00003)
    rng = np.random.default_rng(8427); best = -1; history = []
    for epoch in range(args.road_epochs):
        model.train(); losses = []; order = rng.permutation(np.r_[train, np.tile(sim, 4)])
        for start in range(0, len(order), 16):
            ids = order[start:start + 16]
            inp = torch.as_tensor(x[ids].astype(np.float32), device=device)
            target = torch.as_tensor(y[ids].copy(), device=device)[:, None]
            if rng.random() < .5:
                inp = inp.flip(-1); target = target.flip(-1)
            mask = target != 255; truth = (target == 1).float()
            logits = F.interpolate(model(inp), size=(96, 176), mode='bilinear', align_corners=False)
            bce = F.binary_cross_entropy_with_logits(logits, truth, reduction='none')
            prob = logits.sigmoid() * mask; actual = truth * mask
            dice = 1 - (2 * (prob * actual).sum((1, 2, 3)) + 1) / (prob.sum((1, 2, 3)) + actual.sum((1, 2, 3)) + 1)
            loss = (bce * mask).sum() / mask.sum().clamp_min(1) + .5 * dice.mean()
            opt.zero_grad(set_to_none=True); loss.backward(); torch.nn.utils.clip_grad_norm_(model.parameters(), 1.); opt.step()
            losses.append(float(loss.detach()))
        schedule.step(); score = road_scores(model, x, y, valid, device)
        metrics = {'epoch': epoch + 1, 'loss': float(np.mean(losses)), 'real_validation_pseudo': score}; history.append(metrics)
        print(json.dumps({'stage': 'road', **metrics}), flush=True)
        if score['iou'] > best:
            best = score['iou']; selected = epoch + 1
            torch.save({k: v.cpu() for k, v in model.state_dict().items()}, args.output / 'road_state.pt')
    model.load_state_dict(torch.load(args.output / 'road_state.pt', map_location='cpu', weights_only=True))
    candidates = {str(t): road_scores(model, x, y, valid, device, t) for t in [.4, .5, .6]}
    threshold = float(max(candidates, key=lambda t: candidates[t]['iou']))
    scores = {}
    for source, split in [('real_pseudo', 'val'), ('real_pseudo', 'test'), ('simulation_road', 'val')]:
        ids = np.array([i for i, r in enumerate(rows) if r['source'] == source and r['split'] == split])
        scores[source + '_' + split] = {'frames': len(ids), **road_scores(model, x, y, ids, device, threshold)}
    report = {'selected_epoch': selected, 'initial_readout_sha256': sha256(initial), 'history': history,
        'threshold': threshold, 'threshold_selection': candidates, 'evaluation': scores,
        'ground_truth_note': 'Real targets are R50 pseudo labels; teacher overlap unaudited. Simulation targets are rendered offline.'}
    save_json(args.output / 'road-training.json', report)
    return model, threshold


def train_actor(args, rows, road, x, device, threshold):
    masks = np.lib.format.open_memmap(args.cache / 'predicted_masks.npy', mode='w+', dtype=np.uint8,
                                     shape=(len(rows), 270, 480))
    road.eval()
    with torch.inference_mode():
        for start in range(0, len(rows), 16):
            logits = road(torch.as_tensor(x[start:start + 16].astype(np.float32), device=device))
            masks[start:start + 16] = (F.interpolate(logits, size=(270, 480), mode='bilinear', align_corners=False).sigmoid()[:, 0] >= threshold).cpu().numpy()
    masks.flush(); model = FrontDriveReadout().to(device)
    opt = torch.optim.AdamW(model.parameters(), lr=.001, weight_decay=.0001)
    sch = torch.optim.lr_scheduler.CosineAnnealingLR(opt, args.actor_epochs, eta_min=.00003)
    train = np.array([i for i, r in enumerate(rows) if r['split'] == 'train' and r['command']])
    valid = np.array([i for i, r in enumerate(rows) if r['split'] == 'val' and r['command']])
    sim = np.array([i for i in train if rows[i]['source'] == 'simulation_policy'])
    rng = np.random.default_rng(7314); history = []; best = np.inf

    def batch(ids, random, faults=True):
        values = []; commands = []; stops = []; weights = []
        for i in ids:
            row = rows[i]; c = row['command']; mask = masks[i].copy()
            action = [c['linear_x_mps'] / .8, c['angular_z_radps'] / .35]; stop = int(c['stop'])
            if faults:
                r = random.random()
                if r < .15:
                    mask[:] = 0; stop = 1; action = [0., 0.]
                elif r < .65:
                    noise = SensorFaults({'enabled': True, 'profile': 'normal', 'seed': int(random.integers(1000, 2000))})
                    event = noise.active_event('holes', noise.next_start['holes'])
                    mask = noise.apply(mask[None], (event['start'] + event['end']) / 2)[0][0]
                if random.random() < .5:
                    mask = mask[:, ::-1]; action[1] *= -1
            values.append(front_pixels(mask)[0]); commands.append(action); stops.append(stop)
            weights.append(1. if row['source'] == 'simulation_policy' else .35)
        return [torch.as_tensor(np.asarray(v), device=device) for v in [values, np.array(commands, np.float32), np.array(stops, np.int64), np.array(weights, np.float32)]]

    def evaluate(ids):
        model.eval(); errors = []; correct = []; truth_stop = []
        with torch.inference_mode():
            for start in range(0, len(ids), 64):
                a, b, stop, _ = batch(ids[start:start + 64], np.random.default_rng(2091), False)
                prediction, logits = model(a); move = stop == 0
                errors.extend(((prediction[move] - b[move]).abs() * torch.tensor([.8, .35], device=device)).cpu().tolist())
                correct.extend((logits.argmax(1) == stop).cpu().tolist()); truth_stop.extend(stop.cpu().tolist())
        return {'speed_mae_mps': float(np.mean(errors, axis=0)[0]), 'yaw_mae_radps': float(np.mean(errors, axis=0)[1]),
                'stop_accuracy': float(np.mean(correct)), 'stop_targets': int(sum(truth_stop)), 'frames': len(ids)}

    for epoch in range(args.actor_epochs):
        model.train(); losses = []; order = rng.permutation(np.r_[train, np.tile(sim, 2)])
        for start in range(0, len(order), 48):
            a, b, stop, weight = batch(order[start:start + 48], rng)
            pred, logits = model(a)
            action_loss = ((pred - b) ** 2 * torch.tensor([1., 10.], device=device)).mean(1)
            loss = (action_loss * (stop == 0) * weight).mean() + .3 * F.cross_entropy(logits, stop)
            opt.zero_grad(set_to_none=True); loss.backward(); torch.nn.utils.clip_grad_norm_(model.parameters(), 2.); opt.step()
            losses.append(float(loss.detach()))
        sch.step()
        if epoch % 5 == 0 or epoch == args.actor_epochs - 1:
            score = evaluate(valid); metrics = {'epoch': epoch + 1, 'loss': float(np.mean(losses)), **score}; history.append(metrics)
            print(json.dumps({'stage': 'actor', **metrics}), flush=True)
            value = score['yaw_mae_radps'] + score['speed_mae_mps'] * .1 + (1 - score['stop_accuracy']) * .1
            if value < best:
                best = value; selected = epoch + 1
                torch.save({k: v.cpu() for k, v in model.state_dict().items()}, args.output / 'actor_state.pt')
    model.load_state_dict(torch.load(args.output / 'actor_state.pt', map_location='cpu', weights_only=True))
    report = {'selected_epoch': selected, 'history': history, 'evaluation': {},
        'augmentation': 'fresh random holes and blank masks; blank -> offline conservative stop teacher; no fault identity input',
        'real_labels_weight': .35, 'sim_labels_weight': 1., 'frame_history': 1}
    for source in ['real_pseudo', 'simulation_policy']:
        ids = np.array([i for i in valid if rows[i]['source'] == source])
        report['evaluation'][source] = evaluate(ids)
    save_json(args.output / 'actor-training.json', report)
    return model


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--dataset', type=Path, default=Path('/Volumes/MY_HDD/connectum_data'))
    p.add_argument('--output', type=Path, default=Path('/Volumes/MY_HDD/connectum_models/front-road-v1-20261027'))
    p.add_argument('--cache', type=Path, default=ROOT / '.cache/front-road-v1')
    p.add_argument('--bundle', type=Path, default=ROOT / '.cache/flyvis_encoder.pt')
    p.add_argument('--road-epochs', type=int, default=24); p.add_argument('--actor-epochs', type=int, default=100)
    p.add_argument('--stage', choices=['all', 'features', 'road', 'actor'], default='all')
    p.add_argument('--exclusions', type=Path, help='audited frame exclusion manifest; original dataset is unchanged')
    args = p.parse_args(); args.output.mkdir(parents=True, exist_ok=True)
    torch.manual_seed(8427); np.random.seed(8427); torch.set_num_threads(2); cv2.setNumThreads(2)
    start = time.monotonic(); rows = entries(args.dataset, args.exclusions)
    save_json(args.output / 'training-inputs.json', {'counts': dict(Counter(r['source'] + ':' + r['split'] for r in rows)),
        'dataset_manifest_sha256': sha256(args.dataset / 'dataset.json'), 'front_only': True, 'encoder_frozen': True,
        'real_control_targets': 'unverified pseudo labels, not executed/expert commands',
        'exclusions_sha256': sha256(args.exclusions) if args.exclusions else None,
        'simulation_layout': json.loads((ROOT / 'environments/dangjin/site.json').read_text())['layout_revision']})
    if args.stage in ['all', 'features']:
        build_features(args, rows)
    if args.stage == 'features':
        return
    x = np.load(args.cache / 'features.npy', mmap_mode='r'); y = np.load(args.cache / 'targets.npy', mmap_mode='r')
    device = 'mps' if torch.backends.mps.is_available() else 'cpu'; torch.set_default_device('cpu')
    if args.stage in ['all', 'road']:
        road, threshold = train_road(args, rows, x, y, device)
    else:
        road = RoadReadout().to(device); road.load_state_dict(torch.load(args.output / 'road_state.pt', map_location='cpu', weights_only=True))
        threshold = json.loads((args.output / 'road-training.json').read_text())['threshold']
    if args.stage == 'road':
        return
    actor = train_actor(args, rows, road, x, device, threshold)
    shutil.copyfile(args.bundle, args.output / 'flyvis_encoder.pt')
    torch.save({'schema': SCHEMA, 'encoder_sha256': BUNDLE_SHA,
        'road_state_dict': {k: v.cpu() for k, v in road.state_dict().items()},
        'actor_state_dict': {k: v.cpu() for k, v in actor.state_dict().items()}}, args.output / 'front_model.pt')
    save_json(args.output / 'model.json', {'schema': SCHEMA, 'camera_views': ['front'], 'input_size_wh': [480, 270],
        'input_color_order': 'BGR uint8, then grayscale in Flyvis', 'road_threshold': threshold,
        'max_speed_mps': .8, 'max_yaw_rate_radps': .35, 'stop_threshold': .5,
        'weights_sha256': {n: sha256(args.output / n) for n in ['flyvis_encoder.pt', 'front_model.pt']},
        'encoder_frozen': True, 'training_host': 'macOS', 'encoder_device': 'cpu', 'training_device': device,
        'real_robot_validated': False, 'wall_s': time.monotonic() - start})
    print(json.dumps({'stage': 'complete', 'output': str(args.output), 'wall_s': time.monotonic() - start}), flush=True)


if __name__ == '__main__':
    main()

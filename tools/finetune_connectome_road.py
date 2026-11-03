"""Fine-tune the existing Flyvis road readout on partially labeled Dangjin 360 footage."""
import argparse
from collections import defaultdict
from hashlib import sha256
import json
from pathlib import Path
import shutil

import cv2
import numpy as np
import torch
from torch.nn import functional as F

from connectome_flow import FlyvisFlow
from connectome_road import CircuitFeatures, RoadReadout
from surround_camera import load_profile, profile_sha256, project_panorama


def digest(path):
    return sha256(path.read_bytes()).hexdigest()


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False) + '\n')


def native_mask(path):
    mask = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
    if mask is None or mask.dtype != np.uint8 or mask.shape != (256, 512):
        raise ValueError(f'Invalid annotation: {path}')
    return cv2.resize(mask, (480, 270), interpolation=cv2.INTER_NEAREST_EXACT)


def targets(root, row):
    semantic = native_mask(root / row['semantic_label'])
    label = np.full_like(semantic, 255)
    known = (semantic != 255) & (semantic != 0)
    label[known] = (semantic[known] == 3)
    if row['binary_road_label']:
        binary = native_mask(root / row['binary_road_label'])
        if not np.isin(binary, [0, 1, 255]).all():
            raise ValueError('Unexpected binary label')
        valid = binary != 255
        conflict = known & valid & (label != binary)
        label[valid] = binary[valid]
        label[conflict] = 255
    return label, semantic


def load_real(root, encoder, profile):
    rows = []
    grouped = defaultdict(list)
    for split in ('train', 'validation'):
        path = root / 'splits' / (split + '.jsonl')
        for line in path.read_text().splitlines():
            row = json.loads(line)
            if row['split'] != split or row['annotation_status'] != 'visual_partial':
                raise ValueError(f'Unexpected annotation in {path}')
            if split == 'train':
                if row['recording_group'] != 'ID482' or not 0 <= row['pts_time_sec'] < 1500:
                    raise ValueError('Restricted recording/time in training split')
                t = row['pts_time_sec']
                part = 'fit' if t < 1170 else 'development' if t >= 1210 else 'gap'
            else:
                if row['recording_group'] == 'ID482':
                    raise ValueError('Training recording in evaluation split')
                part = 'heldout'
            row['partition'] = part
            grouped[row['panorama']].append(len(rows))
            rows.append(row)
    if len({r['id'] for r in rows}) != len(rows):
        raise ValueError('Duplicate view')
    features = np.empty((len(rows), 34, 48, 88), np.float32)
    labels, semantic = [], []
    data_hash = sha256()
    for index, (name, ids) in enumerate(grouped.items()):
        path = root / name
        data_hash.update(bytes.fromhex(digest(path)))
        panorama = cv2.imread(str(path))
        sectors = project_panorama(panorama, profile)
        images = [sectors[profile['views']['order'].index(rows[i]['view'])] for i in ids]
        features[ids] = encoder(images)
        if index % 20 == 0:
            print('Extracted panoramas', index + 1, '/', len(grouped), flush=True)
    for row in rows:
        y, s = targets(root, row)
        labels.append(y)
        semantic.append(s)
        data_hash.update(row['id'].encode())
        data_hash.update(y.tobytes())
        data_hash.update(s.tobytes())
    return rows, features, np.stack(labels), np.stack(semantic), data_hash.hexdigest()


def counts(truth, pred):
    return np.bincount((2 * truth + pred).ravel(), minlength=4)


def scores(matrix):
    tn, fp, fn, tp = map(int, matrix)
    def ratio(a, b):
        return a / b if b else None
    road_iou = ratio(tp, tp + fp + fn)
    nonroad_iou = ratio(tn, tn + fp + fn)
    return dict(tn=tn, fp=fp, fn=fn, tp=tp, road_iou=road_iou,
                precision=ratio(tp, tp + fp), recall=ratio(tp, tp + fn),
                nonroad_iou=nonroad_iou, balanced_iou=np.mean([road_iou, nonroad_iou])
                if road_iou is not None and nonroad_iou is not None else None)


@torch.inference_mode()
def evaluate(model, x, y, semantic, ids, rows, threshold):
    model.eval()
    matrices = defaultdict(lambda: np.zeros(4, np.int64))
    vegetation = np.zeros(2, np.int64)
    for start in range(0, len(ids), 8):
        batch = ids[start:start + 8]
        logits = F.interpolate(model(x[batch]), size=(270, 480), mode='bilinear', align_corners=False)
        predictions = (logits[:, 0].sigmoid() >= threshold).cpu().numpy()
        for i, pred in zip(batch, predictions):
            known = y[i] != 255
            cm = counts(y[i][known], pred[known])
            for key in ('all', rows[i]['view'], rows[i]['clip']):
                matrices[key] += cm
            grass = (semantic[i] == 11) & (y[i] == 0)
            vegetation += [np.sum(pred & grass), grass.sum()]
    return dict(by_group={k: scores(v) for k, v in matrices.items()},
                vegetation_road_pixels=int(vegetation[0]), vegetation_pixels=int(vegetation[1]),
                vegetation_false_positive_rate=float(vegetation[0] / max(1, vegetation[1])))


def replay_data(folder, parent):
    # Use the exact cached activities from parent training, and only its training poses.
    with np.load(folder / 'neural-features.npz') as saved:
        if str(saved['dataset_sha256']) != parent['dataset_sha256']:
            raise ValueError('Synthetic replay dataset differs from parent')
        ids = [i for i in range(512) if json.loads((folder / f'{i:03}.json').read_text())['pose_group'] % 5]
        x = saved['features'][ids].astype(np.float32)
    y = [cv2.imread(str(folder / f'{i:03}-road.png'), cv2.IMREAD_GRAYSCALE) for i in ids]
    if not np.isin(np.stack(y), [0, 1]).all():
        raise ValueError('Unexpected synthetic labels')
    return x, np.stack(y)


def pixel_weights(targets, device):
    # Preserve the mass of thin partial polygons when reducing the supervision grid.
    def mask(value):
        weights = [cv2.resize((y == value).astype(np.float32), (176, 96),
                              interpolation=cv2.INTER_AREA) for y in targets]
        return torch.as_tensor(np.stack(weights), device=device)[:, None]
    return mask(1), mask(0)


def loss_for(logits, pos, neg):
    return .5 * ((F.softplus(-logits) * pos).sum() / pos.sum().clamp_min(1.)
                 + (F.softplus(logits) * neg).sum() / neg.sum().clamp_min(1.))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--data', type=Path, required=True)
    p.add_argument('--parent', type=Path, default=Path('.cache/dangjin-360-20261028/road'))
    p.add_argument('--replay', type=Path, default=Path('.cache/dangjin-360-20261028/dangjin-connectome-training'))
    p.add_argument('--bundle', type=Path, default=Path('.cache/flyvis_encoder.pt'))
    p.add_argument('--output', type=Path, required=True, help='New model package directory')
    p.add_argument('--epochs', type=int, default=32)
    p.add_argument('--seed', type=int, default=9271)
    args = p.parse_args()
    out = args.output / 'road'
    out.mkdir(parents=True, exist_ok=False)
    if args.epochs < 1:
        raise ValueError('At least one epoch required')
    torch.manual_seed(args.seed)
    rng = np.random.default_rng(args.seed)
    torch.set_num_threads(2)
    cv2.setNumThreads(2)
    device = 'mps' if torch.backends.mps.is_available() else 'cpu'
    ckpt = torch.load(args.parent / 'road-readout.pt', map_location='cpu', weights_only=True)
    parent = json.loads((args.parent / 'simulation-training.json').read_text())
    profile = load_profile()
    encoder = FlyvisFlow(args.bundle, ckpt['encoder_sha256'], 'cpu')
    rows, features, masks, sem, real_sha = load_real(args.data, CircuitFeatures(encoder), profile)
    x = torch.as_tensor(features, device=device)
    fit, dev, held = ([i for i, row in enumerate(rows) if row['partition'] == k]
                      for k in ('fit', 'development', 'heldout'))
    rx, ry = replay_data(args.replay, parent)
    synth = torch.as_tensor(rx, device=device)
    rp, rn = pixel_weights(ry, device)
    pos, neg = pixel_weights(masks, device)
    model = RoadReadout().to(device)
    model.load_state_dict(ckpt['state_dict'])
    threshold = float(ckpt['threshold'])
    opt = torch.optim.AdamW(model.parameters(), lr=.0002, weight_decay=.0001)
    best = -1.
    history = []
    for epoch in range(1, args.epochs + 1):
        model.train()
        order = rng.permutation(fit).tolist()
        losses = []
        for j in range(0, len(order), 6):
            real = order[j:j + 6]
            replay = rng.choice(len(rx), 2, replace=False).tolist()
            batch = torch.cat([x[real], synth[replay]])
            yes, no = torch.cat([pos[real], rp[replay]]), torch.cat([neg[real], rn[replay]])
            if rng.random() < .5:
                batch, yes, no = (v.flip(-1) for v in (batch, yes, no))
            logits = F.interpolate(model(batch), size=(96, 176), mode='bilinear', align_corners=False)
            n = len(real)
            loss = .75 * loss_for(logits[:n], yes[:n], no[:n]) + .25 * loss_for(logits[n:], yes[n:], no[n:])
            opt.zero_grad(set_to_none=True)
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), 1.)
            opt.step()
            losses.append(loss.item())
        metric = evaluate(model, x, masks, sem, dev, rows, threshold)['by_group']['all']['balanced_iou']
        history.append(dict(epoch=epoch, loss=float(np.mean(losses)), development_balanced_iou=metric))
        print(json.dumps(history[-1]), flush=True)
        if metric > best:
            best, best_epoch = metric, epoch
            state = {k: v.detach().cpu().clone() for k, v in model.state_dict().items()}
        if epoch - best_epoch >= 8:
            break

    def heldout(weights):
        model.load_state_dict(weights)
        return evaluate(model, x, masks, sem, held, rows, threshold)

    results = dict(parent=heldout(ckpt['state_dict']), tuned=heldout(state))
    torch.save(dict(schema=ckpt['schema'], encoder_sha256=ckpt['encoder_sha256'],
                    state_dict=state, threshold=threshold), out / 'road-readout.pt')
    shutil.copyfile(args.bundle, args.output / 'flyvis_encoder.pt')
    split_manifest = [{k: row[k] for k in ('id', 'clip', 'group_id', 'view', 'pts_time_sec', 'partition')}
                      for row in rows]
    write_json(args.output / 'splits.json', split_manifest)
    summary = dict(schema='flyvis.road_readout.real_finetune.v1', data=str(args.data.resolve()), real_data_sha256=real_sha,
                   train_manifest_sha256=digest(args.data / 'splits/train.jsonl'),
                   heldout_manifest_sha256=digest(args.data / 'splits/validation.jsonl'),
                   profile_sha256=profile_sha256(profile), encoder_sha256=ckpt['encoder_sha256'],
                   parent_sha256=digest(args.parent / 'road-readout.pt'), weights_sha256=digest(out / 'road-readout.pt'),
                   source_sha256=digest(Path(__file__)), encoder_frozen=True, new_training='RoadReadout only',
                   threshold=threshold, seed=args.seed, device=device, fit_views=len(fit), development_views=len(dev),
                   gap_views=len(rows)-len(fit)-len(dev)-len(held), heldout_views=len(held), epoch_selected=best_epoch,
                   schedule=dict(lr=.0002, epochs_max=args.epochs, patience=8, batch_real=6, batch_replay=2,
                                 synthetic_loss_weight=.25, replay_sha256=parent['dataset_sha256']),
                   label_policy='semantic road=3, binary road=1, ignore conflicts, void and unannotated pixels',
                   history=history, evaluation=results)
    write_json(out / 'simulation-training.json', summary)
    print('Final heldout', {k: v['by_group']['all'] for k,v in results.items()}, flush=True)


if __name__ == '__main__':
    main()

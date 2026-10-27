#!/usr/bin/env python3
"""Class-wise audit of held-out pseudo commands, including rare stop failures."""
import argparse
import json
from pathlib import Path
import numpy as np
import torch
from front_connectome import FrontDriveReadout, front_pixels, sha256


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--model', type=Path, required=True)
    p.add_argument('--cache', type=Path, default=Path('.cache/front-road-v1'))
    a = p.parse_args(); torch.set_num_threads(2)
    rows = json.loads((a.cache / 'entries.json').read_text())
    masks = np.load(a.cache / 'predicted_masks.npy', mmap_mode='r')
    network = FrontDriveReadout().eval()
    network.load_state_dict(torch.load(a.model / 'front_model.pt', map_location='cpu', weights_only=True)['actor_state_dict'])
    report = {'model_sha256': sha256(a.model / 'front_model.pt'), 'splits': {},
              'labels': 'unverified pseudo commands or offline simulation teacher; not expert truth'}
    for source, split in [('real_pseudo', 'val'), ('real_pseudo', 'test'), ('simulation_policy', 'val')]:
        ids = [i for i, r in enumerate(rows) if r['source'] == source and r['split'] == split and r['command']]
        tp = fp = fn = tn = 0; errors = []; missed = []
        for start in range(0, len(ids), 64):
            chosen = ids[start:start + 64]
            with torch.inference_mode():
                output, logits = network(torch.from_numpy(front_pixels(masks[chosen])))
            stop = logits.softmax(1)[:, 1].numpy() >= .5
            action = output.numpy() * [.8, .35]; action[stop] = 0
            for index, prediction, halted in zip(chosen, action, stop):
                command = rows[index]['command']; truth = bool(command['stop'])
                tp += int(halted and truth); fp += int(halted and not truth)
                fn += int(not halted and truth); tn += int(not halted and not truth)
                if truth and not halted:
                    missed.append({'id': rows[index]['id'], 'predicted_vx': float(prediction[0]),
                                   'predicted_yaw_rate': float(prediction[1])})
                errors.append(np.abs(prediction - [command['linear_x_mps'], command['angular_z_radps']]))
        report['splits'][source + '_' + split] = {'command_frames': len(ids),
            'stop_tp': tp, 'stop_fp': fp, 'stop_fn': fn, 'move_tn': tn,
            'stop_recall': tp / (tp + fn) if tp + fn else None,
            'mae_speed_yaw_including_stops': np.mean(errors, axis=0).tolist() if errors else None,
            'missed_pseudo_stops': missed}
    (a.model / 'validation/readout-holdout.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report))


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Real held-out inference and CPU/MPS portability checks; no robot connection."""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import time

import cv2
import numpy as np
import torch

from front_connectome import FrontConnectome


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--model', type=Path, required=True)
    p.add_argument('--dataset', type=Path, default=Path('/Volumes/MY_HDD/connectum_data'))
    a = p.parse_args(); output = a.model / 'validation'; output.mkdir(exist_ok=True)
    rows = [json.loads(s) for s in (a.dataset / 'samples.jsonl').read_text().splitlines()]
    index = a.model / 'training-samples.json'
    if index.exists():
        allowed = {r['id'] for r in json.loads(index.read_text()) if r['source'] == 'real_pseudo' and r['split'] in ['val', 'test']}
        rows = [r for r in rows if r['id'] in allowed]
    chosen = []
    for session in ['20261021_171114', '20261021_170306']:
        subset = [r for r in rows if r['session'] == session]
        chosen.extend(subset[i] for i in np.linspace(20, len(subset) - 20, 4).astype(int))
    cpu = FrontConnectome(a.model, encoder_device='cpu', head_device='cpu')
    accelerated = FrontConnectome(a.model, encoder_device='cpu', head_device='mps') if torch.backends.mps.is_available() else None
    comparisons = []; previews = []
    for i, row in enumerate(chosen):
        frame = cv2.imread(str(a.dataset / row['camera']['image']))
        mask, command, metrics = cpu.infer(frame)
        record = {'id': row['id'], 'cpu_command': asdict(command), 'cpu_ms': metrics['perception']['ms'] + metrics['policy']['ms']}
        assert np.isin(mask, [0, 1]).all() and np.isfinite([command.vx, command.yaw_rate]).all()
        if accelerated:
            other, action, _ = accelerated.infer(frame)
            record.update(mask_disagreement_fraction=float((other != mask).mean()),
                speed_difference_mps=abs(action.vx - command.vx), yaw_difference_radps=abs(action.yaw_rate - command.yaw_rate),
                stop_equal=(action.vx == 0) == (command.vx == 0))
            assert record['mask_disagreement_fraction'] < .01
            assert record['speed_difference_mps'] < .02 and record['yaw_difference_radps'] < .02 and record['stop_equal']
        comparisons.append(record)
        image = cv2.resize(frame, (480, 270)); image[mask == 1] = (image[mask == 1] * .55 + np.array([140, 210, 70]) * .45).astype(np.uint8)
        tile = np.full((308, 480, 3), 22, np.uint8); tile[:270] = image
        cv2.putText(tile, row['id'], (8, 285), cv2.FONT_HERSHEY_SIMPLEX, .35, (245, 245, 245), 1)
        cv2.putText(tile, f'ESTIMATE v={command.vx:.3f} yaw={command.yaw_rate:+.3f}', (8, 301), cv2.FONT_HERSHEY_SIMPLEX, .35, (100, 230, 255), 1)
        previews.append(tile)
        print(json.dumps(record), flush=True)
    blank = np.zeros((270, 480), np.uint8); stop, detail = cpu.act(blank)
    assert stop.vx == 0 and stop.yaw_rate == 0, 'Learned actor must stop on total detection loss'
    black_mask, black_command, black_metrics = cpu.infer(np.zeros((270, 480, 3), np.uint8))
    montage = np.concatenate([np.concatenate(previews[i:i + 2], axis=1) for i in range(0, len(previews), 2)], axis=0)
    cv2.imwrite(str(output / 'heldout-preview.jpg'), montage)
    report = {'passed': True, 'scope': 'Mac CPU versus MPS inference; held-out real images. Jetson hardware not tested.',
        'comparisons': comparisons, 'blank_detection': {'command': asdict(stop), **detail},
        'black_camera_probe': {'road_fraction': float(black_mask.mean()), 'command': asdict(black_command),
            'stop_test_passed': black_command.vx == 0 and black_command.yaw_rate == 0},
        'model': cpu.details, 'expert_label_accuracy_claimed': False}
    (output / 'portability.json').write_text(json.dumps(report, indent=2) + '\n')


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Read real clips through the consumer loader and prepare a review queue."""
import argparse
from collections import Counter
import json
from pathlib import Path
import time

import numpy as np

from build_connectum_dataset import dump, line
from connectum_dataset_loader import ConnectumDataset


def run(root, watch):
    while not (root / 'dataset.json').exists():
        if not watch:
            raise FileNotFoundError('dataset.json')
        time.sleep(5)
    summary = json.loads((root / 'dataset.json').read_text())
    assert summary['status'] == 'complete' and summary['validation_passed']
    samples = {r['id']: r for r in map(json.loads, (root / 'samples.jsonl').read_text().splitlines())}
    labels = {r['id']: r for r in map(json.loads, (root / 'pseudo_labels.jsonl').read_text().splitlines())}
    clips = list(map(json.loads, (root / 'clips.jsonl').read_text().splitlines()))
    for clip in clips:
        frames = [samples[i] for i in clip['frame_ids']]
        assert len(frames) == 8 and clip['label_frame_id'] == frames[-1]['id']
        assert {r['sequence'] for r in frames} == {clip['sequence']}
        stamps = np.array([int(r['camera']['header_time_ns']) for r in frames], np.int64)
        delta = np.diff(stamps) / 1e9
        assert ((delta > 0) & (delta <= .4)).all()
        np.testing.assert_allclose(delta, clip['dt_s'][1:])
    tests = []
    for split in ['train', 'val', 'test']:
        ds = ConnectumDataset(root, split=split, allow_pseudo=True)
        assert len(ds) > 0
        chosen = {0, len(ds) - 1}
        for mode in ['missing', 'forward', 'turn_left', 'turn_right', 'stop']:
            for i, clip in enumerate(ds.clips):
                cmd = labels[clip['label_frame_id']]['estimated_command']
                if ('missing' if cmd is None else cmd['action']) == mode:
                    chosen.add(i)
                    break
        for i in sorted(chosen):
            item = ds[i]
            assert item['rgb'].shape == (8, 3, 270, 480)
            assert item['rgb'].dtype == np.float32 and np.isfinite(item['rgb']).all()
            assert 0 <= item['rgb'].min() <= item['rgb'].max() <= 1
            assert item['road_target'].shape == (270, 480)
            assert set(np.unique(item['road_target'])) <= {0, 1, 255}
            assert np.array_equal(item['road_valid'], item['road_target'] != 255)
            cmd = labels[item['id']]['estimated_command']
            assert item['action_valid'] == (cmd is not None)
            if cmd:
                np.testing.assert_allclose(item['action_target'],
                    [cmd['linear_x_mps'], cmd['angular_z_radps'], float(cmd['stop'])])
            else:
                assert np.array_equal(item['action_target'], np.zeros(3))
            assert item['is_pseudo'] and not item['human_verified'] and item['state_reset']
            tests.append({'split': split, 'id': item['id'], 'action_valid': item['action_valid'], 'passed': True})
        print(line({'event': 'loader_checked', 'split': split, 'clips': len(ds), 'tested': len(chosen)}), end='', flush=True)
    dump(root / 'loader_smoke_test.json', {'passed': True, 'all_clip_metadata_checked': len(clips),
        'actual_clips_loaded': len(tests), 'tests': tests,
        'scope': 'Actual file reads, RGB ranges/shapes, target masks/units, missing-action loss mask and temporal continuity.'})
    queue = []
    for label in labels.values():
        command = label['estimated_command']
        if command:
            priority = {'stop': 0, 'turn_left': 2, 'turn_right': 2, 'forward': 3}[command['action']]
        else:
            priority = 4 if label['command_status'] == 'abstain_missing_lidar' else 1
        queue.append({'id': label['id'], 'priority': priority, 'human_verified': False,
            'image': samples[label['id']]['camera']['image'], 'command_status': label['command_status'],
            'estimated_command': command, 'labels': label['labels'],
            'review_hint': 'Check raw LiDAR and calibration before accepting stop' if priority == 0 else 'Verify road boundary and intended motion; absence is not a stop label'})
    queue.sort(key=lambda r: (r['priority'], r['id']))
    with (root / 'review_queue.jsonl').open('w') as stream:
        for row in queue:
            stream.write(line(row))
    split_counts = {}
    for split in ['train', 'val', 'test', 'review']:
        selected = [c for c in clips if c['split'] == split]
        actions = Counter(labels[c['label_frame_id']]['estimated_command']['action']
                          for c in selected if c['command_available'])
        split_counts[split] = {'clips': len(selected), 'candidate_actions': dict(actions),
            'missing_action_targets': sum(not c['command_available'] for c in selected)}
    dump(root / 'quality_report.json', {'status': 'pseudo_dataset_requires_review',
        'source_spotcheck_passed': json.loads((root / 'source_spotcheck.json').read_text())['passed'],
        'loader_checks_passed': True, 'split_clip_targets': split_counts,
        'review_queue_entries': len(queue), 'review_queue_priority_counts': dict(Counter(r['priority'] for r in queue)),
        'priority_order': ['0: stop candidates', '1: ambiguous geometry or visual paths',
                           '2: turns', '3: forward', '4: no-LiDAR surface labels'],
        'minimum_cautions': ['No expert-verified commands; every action is a provisional teacher suggestion.',
            'Few stop candidates cannot provide balanced stop/obstacle training.',
            'Test session has no action targets; recognition only.',
            'See visual_review_notes.json for a stop candidate requiring depth review.',
            'This audit does not evaluate model accuracy or real driving performance.']})


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--dataset', type=Path, required=True)
    p.add_argument('--watch', action='store_true')
    a = p.parse_args()
    run(a.dataset, a.watch)

"""Load the real360 preparation package with explicit supervision validity."""
import json
from pathlib import Path

import cv2
import numpy as np


def read_rows(path):
    return [json.loads(line) for line in Path(path).read_text().splitlines()]


def four_views(root, name):
    atlas = cv2.imread(str(root / name))
    if atlas is None or atlas.shape != (540, 960, 3):
        raise ValueError(f'Invalid four-view RGB atlas: {name}')
    return np.stack([atlas[y:y+270, x:x+480] for y, x in ((0, 0), (0, 480), (270, 0), (270, 480))])


class Connectome360RoadDataset:
    def __init__(self, root, split='train'):
        self.root = Path(root)
        self.rows = [r for r in read_rows(self.root/'road/annotations.jsonl') if r['split']==split]

    def __len__(self):
        return len(self.rows)

    def __getitem__(self, index):
        row = self.rows[index]
        views = four_views(self.root, row['views_atlas'])
        road = cv2.imread(str(self.root/row['road_label']), cv2.IMREAD_GRAYSCALE)
        semantic = cv2.imread(str(self.root/row['semantic_label']), cv2.IMREAD_GRAYSCALE)
        if road is None or not np.isin(road, [0, 1, 255]).all():
            raise ValueError('Invalid binary road supervision')
        return dict(id=row['id'], bgr=views[row['view_index']].copy(), road_target=road,
                    road_valid=road!=255, semantic_target=semantic,
                    label_origin=row['label_origin'], human_verified=row['human_verified'])


class Connectome360PolicyDataset:
    def __init__(self, root, split='train', *, allow_estimated_actions=False):
        self.root = Path(root)
        frames = {r['id']: r for r in read_rows(self.root/'frames.jsonl')}
        reviewed_file = self.root/'policy/reviewed-actions.jsonl'
        reviewed = {}
        if reviewed_file.is_file():
            for row in read_rows(reviewed_file):
                source = frames[row['id']]
                command = row['command']
                if (not row['approved'] or not row['human_verified']
                        or row['split']!=source['split']
                        or abs(row['source_time_sec']-source['source_time_sec'])>.000001
                        or not 0 <= command['vx_mps'] <= .8
                        or abs(command['yaw_radps'])>.35):
                    raise ValueError('Invalid reviewed action')
                reviewed[row['id']] = row
        self.rows = []
        for file in sorted((self.root/'policy').glob('[A-Z]*-proposals.jsonl')):
            for row in read_rows(file):
                if row['split']!=split:
                    continue
                if row['id'] in reviewed:
                    accepted = reviewed[row['id']]
                    row = dict(row, estimated_command=accepted['command'],
                               action_valid=True, approved=True, human_verified=True)
                if not row['action_valid'] and not allow_estimated_actions:
                    continue
                self.rows.append((frames[row['id']], row))
        if not self.rows:
            raise ValueError('No verified actions. Review the proposals or explicitly allow_estimated_actions=True.')

    def __len__(self):
        return len(self.rows)

    def __getitem__(self, index):
        frame, row = self.rows[index]
        action = row['estimated_command']
        normalization = row['normalization']
        # Run these pixels through the selected frozen Connectome road detector
        # before policy_pixels(). Cached FCN classes are annotation proposals.
        return dict(id=frame['id'], bgr=four_views(self.root, frame['views_atlas']),
                    action_target=np.array([action['vx_mps']/normalization['vx_mps'],
                                            action['yaw_radps']/normalization['yaw_radps'],
                                            float(action['stop'])], np.float32),
                    action_is_estimated=True, human_verified=row['human_verified'],
                    executed_command_available=False)

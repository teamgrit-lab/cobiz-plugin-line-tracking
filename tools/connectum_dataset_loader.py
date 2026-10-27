"""Camera-only temporal inputs and explicitly opted-in pseudo targets.

NumPy loader; no training, robot connection or control publishing. Compatible
with torch DataLoader's mapping/NumPy collation if used by a training script.
"""
from collections import OrderedDict
import json
from pathlib import Path

import cv2
import numpy as np


class ConnectumDataset:
    def __init__(self,root,split='train',*,allow_pseudo=False,size_wh=(480,270),shard_cache_size=2):
        if not allow_pseudo:raise ValueError('This dataset has pseudo labels only. Explicitly set allow_pseudo=True to use them.')
        self.root=Path(root);self.size_wh=size_wh;self.cache_size=shard_cache_size;self.cache=OrderedDict()
        self.samples={r['id']:r for r in map(json.loads,(self.root/'samples.jsonl').read_text().splitlines())}
        self.labels={r['id']:r for r in map(json.loads,(self.root/'pseudo_labels.jsonl').read_text().splitlines())}
        self.clips=[c for c in map(json.loads,(self.root/'clips.jsonl').read_text().splitlines()) if c['split']==split]

    def __len__(self):return len(self.clips)

    def _array(self,ref):
        path=ref['archive']
        if path not in self.cache:
            with np.load(self.root/path) as z:self.cache[path]={k:z[k] for k in ['road','confidence']}
            while len(self.cache)>self.cache_size:self.cache.popitem(last=False)
        self.cache.move_to_end(path)
        return self.cache[path][ref['array']][ref['index']]

    def __getitem__(self,index):
        clip=self.clips[index];label=self.labels[clip['label_frame_id']]
        frames=[]
        for sid in clip['frame_ids']:
            image=cv2.imread(str(self.root/self.samples[sid]['camera']['image']))
            image=cv2.cvtColor(cv2.resize(image,self.size_wh),cv2.COLOR_BGR2RGB)
            frames.append(image.transpose(2,0,1).astype(np.float32)/255.)
        mask=cv2.resize(self._array(label['labels']['road']),self.size_wh,interpolation=cv2.INTER_NEAREST).astype(np.int64)
        command=label['estimated_command'];valid=command is not None
        # This placeholder MUST be excluded by action_valid; absence is not stop.
        action=np.array([command['linear_x_mps'],command['angular_z_radps'],float(command['stop'])] if valid else [0.,0.,0.],np.float32)
        return {'id':clip['id'],'rgb':np.stack(frames),'dt_s':np.array(clip['dt_s'],np.float32),
            'road_target':mask,'road_valid':mask!=255,'action_target':action,'action_valid':valid,
            'is_pseudo':True,'human_verified':False,'state_reset':True}

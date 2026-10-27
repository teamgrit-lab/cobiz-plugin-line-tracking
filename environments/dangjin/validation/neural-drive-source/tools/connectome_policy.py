"""Learned velocity readout of the Flyvis road bottleneck, simulation only.

The actor receives only the four inferred (possibly damaged) masks and a local
goal direction. It does not receive pose, the route, fault flags or a planner
command. Its categorical stop decision is learned, not a mask-loss override.
"""
from pathlib import Path
import hashlib
import json
import time

import cv2
import numpy as np
import torch
from torch import nn

from swin_l_drive_control import DriveDecision


def policy_pixels(masks):
    if np.asarray(masks).shape!=(4,270,480):
        raise ValueError('four aligned camera road masks required')
    # Fixed image resampling, not a geometric path or centerline computation.
    return np.stack([cv2.resize(m[135:257].astype(np.float32),(64,32),interpolation=cv2.INTER_AREA) for m in masks])


class DriveReadout(nn.Module):
    def __init__(self):
        super().__init__()
        self.vision=nn.Sequential(nn.Conv2d(4,16,5,stride=2,padding=2),nn.SiLU(),
            nn.Conv2d(16,24,3,stride=2,padding=1),nn.SiLU(),
            nn.Conv2d(24,32,3,stride=2,padding=1),nn.SiLU(),nn.Flatten(),
            nn.Linear(32*4*8,64),nn.SiLU())
        self.action=nn.Sequential(nn.Linear(66,64),nn.SiLU(),nn.Linear(64,32),nn.SiLU(),nn.Linear(32,2))
        self.stop=nn.Sequential(nn.Linear(64,32),nn.SiLU(),nn.Linear(32,2))

    def forward(self,masks,goal):
        visual=self.vision(masks)
        raw=self.action(torch.cat([visual,goal],dim=1))
        return torch.stack([raw[:,0].sigmoid(),raw[:,1].tanh()],1),self.stop(visual)


class ConnectomeDrivePolicy:
    def __init__(self,path,perception,device='cpu'):
        path=Path(path);checkpoint=torch.load(path/'drive-readout.pt',map_location='cpu',weights_only=True)
        if checkpoint['road_sha256']!=perception.details['weights_sha256']:
            raise ValueError('drive policy requires its trained Flyvis road model')
        self.model=DriveReadout().to(device).eval().requires_grad_(False)
        self.model.load_state_dict(checkpoint['state_dict'],strict=True);self.device=device
        self.details={'backend':'flyvis-drive-readout','schema':'dangjin.neural_policy.v1',
            'weights_sha256':hashlib.sha256((path/'drive-readout.pt').read_bytes()).hexdigest(),
            'road_sha256':checkpoint['road_sha256'],'encoder_sha256':perception.details['encoder_sha256'],
            'input':'four damaged Flyvis road masks plus local route goal',
            'outputs':['vx','yaw_rate','learned_stop'],'runtime_planner':False,'runtime_pure_pursuit':False,
            'emergency_stop_override':False,'fault_flag_input':False,'pose_input':False,
            'map_assistance':'local goal in body coordinates; no map pixels or reference steering',
            'connectome_frozen':True,'simulation_only':True,
            'training':json.loads((path/'training.json').read_text())}

    def infer(self,masks,goal):
        start=time.perf_counter()
        pixels=policy_pixels(masks)
        goal=np.asarray(goal,np.float32)
        if goal.shape!=(2,) or not np.isfinite(goal).all():raise ValueError('finite local goal required')
        with torch.inference_mode():
            action,logits=self.model(torch.as_tensor(pixels[None],device=self.device),torch.as_tensor(goal[None]/6,device=self.device))
            action=action[0].cpu().numpy();prob=logits.softmax(1)[0].cpu().numpy()
        stopped=bool(prob[1]>prob[0])
        command=DriveDecision(0. if stopped else float(.8*action[0]),0.,0. if stopped else float(.35*action[1]),
            'neural_wait' if stopped else 'neural_drive')
        return command,{'ms':(time.perf_counter()-start)*1000,'stop_probability':float(prob[1]),
            'raw_action':action.tolist(),'local_goal':goal.tolist(),'override':False}

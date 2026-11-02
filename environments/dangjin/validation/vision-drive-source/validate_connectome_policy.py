#!/usr/bin/env python3
"""Causal input checks for the saved, trained simulation action readout."""
import hashlib
import json
from pathlib import Path

import numpy as np
import torch

from connectome_policy import DriveReadout,policy_pixels
from train_connectome_policy import teacher


def main():
    torch.set_num_threads(2)
    root=Path('.cache/dangjin-connectome-policy-vision');data=Path('.cache/dangjin-policy-training')
    checkpoint=torch.load(root/'drive-readout.pt',map_location='cpu',weights_only=True)
    model=DriveReadout().eval().requires_grad_(False);model.load_state_dict(checkpoint['state_dict'])
    training=json.loads((root/'training.json').read_text())
    metadata=[json.loads(f.read_text()) for f in sorted(data.glob('[0-9][0-9][0-9][0-9].json'))]
    indices=[i for i,m in enumerate(metadata) if not m['pose_group']%5]
    masks=np.load(data/'inferred-road-masks.npz')['masks'][indices]
    pixels=torch.as_tensor(np.stack([policy_pixels(m) for m in masks]))
    route=np.asarray(json.loads(Path('environments/dangjin/world.json').read_text())['perimeter_route'])
    targets=np.stack([teacher(metadata[i],route) for i in indices])
    with torch.inference_mode():
        normal,normal_stop=model(pixels)
        blank,blank_stop=model(torch.zeros_like(pixels))
        shuffled,shuffle_stop=model(pixels.flip(0))
    normal=normal.numpy();shuffled=shuffled.numpy()
    delta=float(np.mean(np.abs(normal[:,1]-shuffled[:,1]))*.35)
    assert delta>1e-4,'steering must depend on visual input without any map input'
    assert bool((blank_stop.argmax(1)==1).all()),'learned blank-view stop failed'
    assert bool((normal_stop.argmax(1)==0).all()),'clear held-out view incorrectly stopped'
    report={'schema':'dangjin.neural_drive.causal_validation.v1','heldout_samples':len(indices),
        'weights_sha256':hashlib.sha256((root/'drive-readout.pt').read_bytes()).hexdigest(),
        'training_selected_epoch':training['selected_epoch'],
        'clean_speed_mae_mps':float(np.mean(abs(normal[:,0]-targets[:,0]))*.8),
        'clean_yaw_mae_rad_s':float(np.mean(abs(normal[:,1]-targets[:,1]))*.35),
        'blank_input_learned_stop_rate':float((blank_stop.argmax(1)==1).float().mean()),
        'clear_input_drive_rate':float((normal_stop.argmax(1)==0).float().mean()),
        'shuffled_vision_yaw_change_rad_s':delta,
        'shuffled_vision_yaw_mae_rad_s':float(np.mean(abs(shuffled[:,1]-targets[:,1]))*.35),
        'goal_input':False,'map_assistance':False,
        'fault_flag_input':False,'rule_based_blank_stop':False,'runtime_planner':False,
        'scope':'saved action weights on held-out simulated views; full-lap checks separate'}
    output=Path('environments/dangjin/validation/vision-drive-contracts.json')
    output.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))


if __name__=='__main__':main()

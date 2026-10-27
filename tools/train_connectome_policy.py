#!/usr/bin/env python3
"""Imitate a simulation teacher with a learned Flyvis-mask -> v/omega readout.

Teacher geometry is used offline only. All variants at a held-out route station
stay outside training. Fault seeds 1000..1999 train, 2000..2999 validate; complete
closed-loop browser evaluation must use other seeds and newly rendered frames.
"""
import argparse
import hashlib
import json
from pathlib import Path
import time

import cv2
import numpy as np
import torch
from torch.nn import functional as F

from connectome_flow import FlyvisFlow
from connectome_road import ConnectomeRoadPerception
from connectome_policy import DriveReadout,policy_pixels
from dangjin_patrol import track_camera_path
from dangjin_sensor_faults import SensorFaults
from dangjin_sim_server import BUNDLE_SHA256


def teacher(metadata,route):
    pose=metadata['pose'];p=np.array([pose['x'],pose['y']]);heading=pose['heading']
    # Offline reference centerline labels, never called by the deployed actor.
    delta=route-p;nearest=int(np.argmin(np.linalg.norm(delta,axis=1)))
    points=route[(nearest+np.arange(0,61))%len(route)]-p
    c,s=np.cos(heading),np.sin(heading);rotation=np.array([[c,-s],[s,c]])
    local=points@rotation
    command=track_camera_path(local,0,.8)
    return np.array([command.vx/.8,command.yaw_rate/.35],np.float32)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--data',type=Path,default=Path('.cache/dangjin-policy-training'))
    p.add_argument('--output',type=Path,default=Path('.cache/dangjin-connectome-policy-vision'))
    p.add_argument('--epochs',type=int,default=120)
    a=p.parse_args();start=time.monotonic();rng=np.random.default_rng(731);torch.manual_seed(731)
    torch.set_num_threads(2);cv2.setNumThreads(2)
    metadata=[json.loads(f.read_text()) for f in sorted(a.data.glob('[0-9][0-9][0-9][0-9].json'))]
    if len(metadata)!=768:raise ValueError(f'768 four-view examples required, got {len(metadata)}')
    route=np.asarray(json.loads(Path('environments/dangjin/world.json').read_text())['perimeter_route'])
    targets=np.asarray([teacher(m,route) for m in metadata])
    road_path=Path('.cache/dangjin-connectome-road');road_sha=hashlib.sha256((road_path/'road-readout.pt').read_bytes()).hexdigest()
    cache=a.data/'inferred-road-masks.npz'
    digest=hashlib.sha256()
    for m in metadata:
        digest.update(json.dumps(m,sort_keys=True).encode())
        for sector in range(4):digest.update((a.data/f'{m["index"]:04d}-{sector}-rgb.png').read_bytes())
    data_sha=digest.hexdigest()
    if cache.exists() and str(np.load(cache)['dataset_sha256'])==data_sha and str(np.load(cache)['road_sha256'])==road_sha:
        masks=np.load(cache)['masks']
    else:
        encoder=FlyvisFlow('.cache/flyvis_encoder.pt',BUNDLE_SHA256,'cpu')
        perception=ConnectomeRoadPerception(encoder,road_path,'mps');masks=[]
        for i,m in enumerate(metadata):
            frames=[cv2.imread(str(a.data/f'{m["index"]:04d}-{sector}-rgb.png')) for sector in range(4)]
            inferred,_=perception.predict_views(frames);masks.append(inferred)
            if i%32==0:print(json.dumps({'extract':i+1,'total':len(metadata)}),flush=True)
        masks=np.asarray(masks);np.savez_compressed(cache,masks=masks,dataset_sha256=data_sha,road_sha256=road_sha)
        del perception,encoder
    train=np.array([i for i,m in enumerate(metadata) if m['pose_group']%5])
    valid=np.array([i for i,m in enumerate(metadata) if not m['pose_group']%5])
    device='mps' if torch.backends.mps.is_available() else 'cpu'
    torch.set_default_device('cpu');model=DriveReadout().to(device)
    optimizer=torch.optim.AdamW(model.parameters(),lr=.001,weight_decay=.0001)
    scheduler=torch.optim.lr_scheduler.CosineAnnealingLR(optimizer,a.epochs,eta_min=.00005)
    a.output.mkdir(exist_ok=True);history=[];best=float('inf')
    # Reuse held-out corruptions exactly across epochs; training draws fresh ones.
    def batch(ids,random,seed_low=1000):
        x=[];y=targets[ids].copy();stop=np.zeros(len(ids),np.int64)
        for j,i in enumerate(ids):
            sample=masks[i]
            r=random.random()
            if r<.16:sample=np.zeros_like(sample);stop[j]=1;y[j]=0
            elif r<.8:
                seed=int(random.integers(seed_low,seed_low+1000))
                noise=SensorFaults({'enabled':True,'profile':'severe' if r<.5 else 'normal','seed':seed})
                # Select a hole burst, without consulting its identity in the actor.
                event=noise.active_event('holes',3.)
                if event is None:event=noise.events['holes'][0]
                sample,_=noise.apply(sample,(event['start']+event['end'])/2)
                if not sample.any():stop[j]=1;y[j]=0
            x.append(policy_pixels(sample))
        x=np.asarray(x)
        mirror=random.random(len(ids))<.5
        x[mirror]=x[mirror][:,[0,3,2,1],:,:,][:,:,:,::-1]
        y[mirror,1]*=-1
        return tuple(torch.as_tensor(v.copy(),device=device) for v in (x,y.astype(np.float32),stop))
    def score():
        model.eval();errors=[];acc=[];prob=[]
        with torch.inference_mode():
            vrng=np.random.default_rng(2047)
            for offset in range(0,len(valid),48):
                x,y,z=batch(valid[offset:offset+48],vrng,2000);pred,logits=model(x)
                errors.append(((pred-y).abs()*(z==0)[:,None]).sum(0).cpu().numpy())
                acc.extend((logits.argmax(1)==z).cpu().tolist());prob.extend(z.cpu().tolist())
        error=np.sum(errors,axis=0)/max(1,len(prob)-sum(prob))
        return {'speed_mae_mps':float(error[0]*.8),'yaw_mae_rad_s':float(error[1]*.35),'stop_accuracy':float(np.mean(acc))}
    for epoch in range(a.epochs):
        model.train();losses=[]
        for ids in np.array_split(rng.permutation(np.tile(train,2)),max(1,len(train)*2//48)):
            x,y,z=batch(ids,rng);pred,logits=model(x)
            loss=(((pred-y)**2)*torch.tensor([1.,5.],device=device)*(z==0)[:,None]).mean()+F.cross_entropy(logits,z)*.25
            optimizer.zero_grad(set_to_none=True);loss.backward();torch.nn.utils.clip_grad_norm_(model.parameters(),2);optimizer.step();losses.append(float(loss.detach()))
        scheduler.step();metrics={'epoch':epoch+1,'loss':float(np.mean(losses)),**score()};history.append(metrics)
        if epoch%5==0:print(json.dumps(metrics),flush=True)
        value=metrics['yaw_mae_rad_s']+metrics['speed_mae_mps']*.1+(1-metrics['stop_accuracy'])
        if value<best:
            best=value;selected=metrics
            torch.save({'schema':'dangjin.drive_readout.vision.v2','road_sha256':road_sha,'state_dict':{k:v.cpu() for k,v in model.state_dict().items()}},a.output/'drive-readout.pt')
    manifest={'schema':'dangjin.drive_training.vision.v2','input':'four camera-derived road masks only','goal_input':False,'seed':731,'train_poses':len(train),'heldout_poses':len(valid),
        'camera_views_per_pose':4,'split':'pose_group % 5 == 0 held out, including every offset variant',
        'dataset_sha256':data_sha,'road_sha256':road_sha,'encoder_sha256':BUNDLE_SHA256,
        'encoder_frozen':True,'road_readout_frozen':True,'trained_parameters':'convolutional drive readout and stop head',
        'offline_teacher':'pure pursuit of simulation route; never used at runtime',
        'fault_seed_range':[1000,1999],'validation_fault_seed_range':[2000,2999],'selected_epoch':selected,'history':history,
        'layout_revision':metadata[0]['layout_revision'],'wall_s':time.monotonic()-start,
        'scope':'synthetic scene imitation learning; needs independent closed-loop tests'}
    (a.output/'training.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(json.dumps({'saved':str(a.output),'selected':selected}),flush=True)


if __name__=='__main__':main()

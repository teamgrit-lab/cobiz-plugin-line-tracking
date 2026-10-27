#!/usr/bin/env python3
"""Train a road readout solely from frozen Flyvis circuit activity.

All four camera directions at a held-out pose stay outside the training split.
This validates a synthetic scene, not real MCAP imagery or a physical robot.
"""
import argparse
import hashlib
import json
from pathlib import Path
import random
import time

import cv2
import numpy as np
import torch
from torch.nn import functional as F

from connectome_flow import FlyvisFlow
from connectome_road import CircuitFeatures,RoadReadout
from dangjin_sim_server import BUNDLE_SHA256


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--data',type=Path,default=Path('.cache/dangjin-connectome-training'))
    p.add_argument('--output',type=Path,default=Path('.cache/dangjin-connectome-road'))
    p.add_argument('--bundle',type=Path,default=Path('.cache/flyvis_encoder.pt'))
    p.add_argument('--epochs',type=int,default=60)
    a=p.parse_args();started=time.monotonic()
    random.seed(47);np.random.seed(47);torch.manual_seed(47);torch.set_num_threads(2);cv2.setNumThreads(2)
    images=sorted(a.data.glob('*-rgb.png'))
    if len(images)!=512:raise ValueError('512 four-direction views required')
    digest=hashlib.sha256();metadata=[];targets=[]
    for index,image in enumerate(images):
        label=image.with_name(image.name.replace('-rgb','-road'));meta=a.data/f'{index:03d}.json'
        for path in [image,label,meta]:digest.update(path.read_bytes())
        metadata.append(json.loads(meta.read_text()))
        mask=cv2.imread(str(label),0)
        if not np.isin(mask,[0,1]).all():raise ValueError('binary offline labels required')
        targets.append(cv2.resize(mask,(176,96),interpolation=cv2.INTER_NEAREST))
    revisions={m['layout_revision'] for m in metadata}
    if len(revisions)!=1:raise ValueError('mixed world revisions')
    dataset_sha=digest.hexdigest();cache=a.data/'neural-features.npz'
    encoder=FlyvisFlow(a.bundle,BUNDLE_SHA256,'cpu');extract=CircuitFeatures(encoder)
    if cache.exists() and str(np.load(cache)['dataset_sha256'])==dataset_sha:
        features=np.load(cache)['features'].astype(np.float32)
    else:
        features=[]
        for start in range(0,len(images),8):
            features.append(extract([cv2.imread(str(i)) for i in images[start:start+8]]))
            if start%64==0:print(json.dumps({'extracted':min(start+8,len(images)),'total':512}),flush=True)
        features=np.concatenate(features)
        np.savez_compressed(cache,features=features.astype(np.float16),dataset_sha256=dataset_sha)
    negative=extract([np.full((270,480,3),value,np.uint8) for value in [0,32,64,96,128,160,192,255]])
    train=[i for i,m in enumerate(metadata) if m['pose_group']%5];valid=[i for i,m in enumerate(metadata) if not m['pose_group']%5]
    train_images=len(train);train+=list(range(512,520))
    features=np.concatenate((features,negative));targets=np.concatenate((targets,np.zeros((8,96,176),np.uint8)))
    device='mps' if torch.backends.mps.is_available() else 'cpu'
    x=torch.as_tensor(features,device=device);y=torch.as_tensor(targets,device=device,dtype=torch.float32)[:,None]
    model=RoadReadout().to(device)
    with torch.no_grad():
        model.mean.copy_(x[train].mean(dim=(0,2,3),keepdim=True))
        model.std.copy_(x[train].std(dim=(0,2,3),keepdim=True).clamp_min(1e-4))
    optimizer=torch.optim.AdamW(model.parameters(),lr=1e-3,weight_decay=1e-4)
    scheduler=torch.optim.lr_scheduler.CosineAnnealingLR(optimizer,a.epochs,eta_min=1e-4)
    a.output.mkdir(parents=True,exist_ok=True);history=[];best=-1;threshold=.55

    def score(ids,ablation=None):
        tp=fp=fn=0
        model.eval()
        with torch.inference_mode():
            for start in range(0,len(ids),16):
                batch=ids[start:start+16];value=x[batch]
                if ablation=='mean_activity':value=model.mean.expand_as(value)
                if ablation=='spatial_shuffle':value=value.flatten(2)[:,:,torch.randperm(value.shape[-2]*value.shape[-1],device=device)].reshape_as(value)
                logits=F.interpolate(model(value),size=(96,176),mode='bilinear',align_corners=False)
                pred=logits.sigmoid()>=threshold;truth=y[batch]>.5
                tp+=int((pred&truth).sum());fp+=int((pred&~truth).sum());fn+=int((~pred&truth).sum())
        return {'road_iou':tp/max(1,tp+fp+fn),'precision':tp/max(1,tp+fp),'recall':tp/max(1,tp+fn)}

    for epoch in range(a.epochs):
        model.train();random.shuffle(train);losses=[]
        for start in range(0,len(train),16):
            batch=train[start:start+16];value=x[batch];truth=y[batch]
            if random.random()<.5:value=value.flip(-1);truth=truth.flip(-1)
            optimizer.zero_grad(set_to_none=True)
            logits=F.interpolate(model(value),size=(96,176),mode='bilinear',align_corners=False)
            prob=logits.sigmoid()
            dice=1-(2*(prob*truth).sum((1,2,3))+1)/(prob.sum((1,2,3))+truth.sum((1,2,3))+1)
            loss=F.binary_cross_entropy_with_logits(logits,truth)+dice.mean()*.5
            loss.backward();torch.nn.utils.clip_grad_norm_(model.parameters(),1.);optimizer.step();losses.append(float(loss.detach()))
        scheduler.step();metrics={'epoch':epoch+1,'loss':float(np.mean(losses)),**score(valid)}
        history.append(metrics);print(json.dumps(metrics),flush=True)
        if metrics['road_iou']>best:
            best=metrics['road_iou'];selected=metrics
            torch.save({'schema':'flyvis.road_readout.v1','encoder_sha256':BUNDLE_SHA256,'state_dict':{k:v.cpu() for k,v in model.state_dict().items()},'threshold':threshold},a.output/'road-readout.pt')
    model.load_state_dict(torch.load(a.output/'road-readout.pt',map_location=device,weights_only=True)['state_dict'])
    ablation={kind:score(valid,kind) for kind in ['mean_activity','spatial_shuffle']}
    per_view={str(sector):score([i for i in valid if i%4==sector]) for sector in range(4)}
    manifest={'schema':'flyvis.road_readout.training.v1','encoder_sha256':BUNDLE_SHA256,'encoder_frozen':True,
        'encoder_pretraining_task':'optical_flow','new_training':'supervised convolutional road readout, not biological weights',
        'input':'34 output cell types x 721 hexals after 10 native 0.02 s steps; linearly interpolated to 48x88',
        'output_cell_types':extract.cell_types,'raw_image_skip_connection':False,'runtime_ground_truth':False,
        'seed':47,'train_views':train_images,'heldout_views':len(valid),'negative_training_views':8,
        'split':'pose_group % 5 == 0 held out; all four directions grouped by pose',
        'layout_revision':next(iter(revisions)),'dataset_sha256':dataset_sha,
        'selected_epoch':selected,'threshold':threshold,'per_view_heldout':per_view,'activity_ablation':ablation,
        'history':history,'wall_s':time.monotonic()-started,
        'scope':'held-out views in this synthetic scene only; no real-world road-detection validation'}
    (a.output/'simulation-training.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(json.dumps({'saved':str(a.output),'selected':selected,'ablation':ablation}),flush=True)


if __name__=='__main__':main()

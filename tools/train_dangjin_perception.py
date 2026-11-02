#!/usr/bin/env python3
"""Adapt pretrained road segmentation on simulator RGB/labels, with held-out views.

Labels are for offline supervised training only. Runtime accepts RGB alone.
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
from transformers import SegformerForSemanticSegmentation
from dangjin_perception import MODEL_ID,REVISION


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source',type=Path,default=Path('.cache/dangjin-segformer-b0'))
    p.add_argument('--data',type=Path,default=Path('.cache/dangjin-training'))
    p.add_argument('--output',type=Path,default=Path('.cache/dangjin-road-model'))
    p.add_argument('--epochs',type=int,default=10);a=p.parse_args()
    random.seed(31);np.random.seed(31);torch.manual_seed(31);torch.set_num_threads(2)
    device='mps' if torch.backends.mps.is_available() else 'cpu'
    model=SegformerForSemanticSegmentation.from_pretrained(a.source,local_files_only=True).to(device)
    images=[];labels=[];hashes=[]
    for image in sorted(a.data.glob('*-rgb.png')):
        mask=image.with_name(image.name.replace('-rgb','-road'))
        rgb=cv2.cvtColor(cv2.resize(cv2.imread(str(image)),(448,256)),cv2.COLOR_BGR2RGB)
        y=cv2.resize(cv2.imread(str(mask),0),(448,256),interpolation=cv2.INTER_NEAREST)
        images.append(rgb);labels.append(y);hashes.append(hashlib.sha256(image.read_bytes()+mask.read_bytes()).hexdigest())
    if len(images)!=128:raise ValueError('Expected exactly 128 collected views')
    x=torch.tensor(np.asarray(images),device=device,dtype=torch.float32).permute(0,3,1,2)/255
    y=torch.tensor(1-np.asarray(labels),device=device,dtype=torch.long)
    mean=torch.tensor([.485,.456,.406],device=device)[None,:,None,None]
    std=torch.tensor([.229,.224,.225],device=device)[None,:,None,None]
    train=[i for i in range(len(x)) if i%5];valid=[i for i in range(len(x)) if not i%5]
    optimizer=torch.optim.AdamW(model.parameters(),lr=7e-5,weight_decay=.01)
    best=-1;history=[];started=time.monotonic();a.output.mkdir(exist_ok=True)
    def binary_logits(logits):
        logits=torch.cat((logits[:,0:1],torch.logsumexp(logits[:,1:],dim=1,keepdim=True)),1)
        return torch.nn.functional.interpolate(logits,size=(256,448),mode='bilinear',align_corners=False)
    for epoch in range(a.epochs):
        model.train();random.shuffle(train);losses=[]
        for begin in range(0,len(train),4):
            ids=train[begin:begin+4];rgb=x[ids];target=y[ids]
            if random.random()<.5:rgb=rgb.flip(-1);target=target.flip(-1)
            rgb=(rgb**random.uniform(.85,1.15)*random.uniform(.9,1.1)).clamp(0,1)
            optimizer.zero_grad(set_to_none=True)
            logits=binary_logits(model(pixel_values=(rgb-mean)/std).logits)
            loss=torch.nn.functional.cross_entropy(logits,target)
            loss.backward();torch.nn.utils.clip_grad_norm_(model.parameters(),1.);optimizer.step();losses.append(float(loss.detach()))
        model.eval();intersection=union=tp=fp=fn=0
        with torch.inference_mode():
            for i in valid:
                prob=binary_logits(model(pixel_values=(x[i:i+1]-mean)/std).logits).softmax(1)[0,0]
                pred=prob>=.55;truth=y[i]==0
                tp+=int((pred&truth).sum());fp+=int((pred&~truth).sum());fn+=int((~pred&truth).sum())
        iou=tp/(tp+fp+fn);metrics={'epoch':epoch+1,'loss':float(np.mean(losses)),'heldout_road_iou':iou,'precision':tp/(tp+fp),'recall':tp/(tp+fn)}
        history.append(metrics);print(json.dumps(metrics),flush=True)
        if iou>best:
            best=iou;model.save_pretrained(a.output,safe_serialization=True);selected=metrics
    manifest={'source_model':MODEL_ID,'source_revision':REVISION,'source_sha256':hashlib.sha256((a.source/'pytorch_model.bin').read_bytes()).hexdigest(),
              'seed':31,'train_views':len(train),'heldout_views':len(valid),'split':'index % 5 == 0 held out','selected_epoch':selected,
              'input':'RGB 448x256','labels':'offline rendered road, including flush manhole lids','runtime_ground_truth':False,
              'dataset_sha256':hashlib.sha256(''.join(hashes).encode()).hexdigest(),'history':history,'wall_s':time.monotonic()-started,
              'scope':'this synthetic scene; no real-world road-recognition validation'}
    (a.output/'simulation-training.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(f'Saved {a.output}; held-out road IoU {best:.4f}',flush=True)


if __name__=='__main__':main()

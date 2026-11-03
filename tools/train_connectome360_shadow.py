"""Shadow fine-tuning of the frozen-Flyvis road readout, preserving label splits."""
from collections import defaultdict
import hashlib
import json
from pathlib import Path
import time

import cv2
import numpy as np
import torch
from torch.nn import functional as F

from connectome360_dataset import four_views, read_rows
from connectome_road import CircuitFeatures, RoadReadout
from connectome_shadow import augment_views, SCHEMA
from finetune_connectome_road import evaluate, pixel_weights, loss_for
from surround_camera import load_profile, profile_sha256


def digest(path):
    with Path(path).open('rb') as handle:
        return hashlib.file_digest(handle,'sha256').hexdigest()


def save(path, value):
    path.write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False)+'\n')


class FeatureReplay:
    def __init__(self,path,annotation,encoder_sha,annotation_sha):
        with np.load(path) as stored:
            assert str(stored['encoder_sha256'])==encoder_sha
            assert str(stored['annotation_sha256'])==annotation_sha
            self.data={k:stored[k] for k in ['real','real_shadow','synthetic','synthetic_shadow','weak','weak_shadow']}
        def chunks():
            j=0
            for i,row in enumerate(annotation):
                yield self.data['real'][i:i+1]
                for variant in range(3 if row['split']=='train' else 1):
                    yield self.data['real_shadow'][j:j+1];j+=1
            for first in range(0,512,4):
                yield self.data['synthetic'][first:first+4]
                yield self.data['synthetic_shadow'][first:first+4]
            for first in range(0,256,4):
                yield self.data['weak'][first:first+4]
                yield self.data['weak_shadow'][first:first+4]
        self.chunks=iter(chunks())

    def __call__(self,frames):
        values=next(self.chunks)
        assert len(values)==len(frames)
        return values.astype(np.float32)

    def finish(self):
        assert next(self.chunks,None) is None


def road_training_shadow(args,encoder,package,device):
    started=time.monotonic()
    extract=CircuitFeatures(encoder)
    annotation=read_rows(args.data/'road/annotations.jsonl')
    cache=getattr(args,'shadow_feature_cache',None)
    cached_records=None
    if cache:
        extract=FeatureReplay(cache,annotation,encoder.details['bundle_sha256'],digest(args.data/'road/annotations.jsonl'))
        cached_records=json.loads((package/'training/shadow-augmentation.json').read_text())['variants']
    real_x,real_y,real_sem,rows=[],[],[],[]
    real_aug,real_sources=[],[]
    records=[]
    for i,row in enumerate(annotation):
        image=four_views(args.data,row['views_atlas'])[row['view_index']]
        real_x.append(extract([image])[0])
        real_y.append(cv2.imread(str(args.data/row['road_label']),0))
        real_sem.append(cv2.imread(str(args.data/row['semantic_label']),0))
        rows.append(dict(row,clip=row['recording']))
        variants=3 if row['split']=='train' else 1
        namespace={'train':10000,'development':20000,'validation':30000,'gap':40000}[row['split']]
        for variant in range(variants):
            changed,params=augment_views([image],seed=args.seed+namespace,key=row['views_atlas'],
                                         variant=variant,view_indices=[row['view_index']])
            real_aug.append(extract(changed)[0]);real_sources.append(i)
            records.append(dict(kind='real',source=row['id'],split=row['split'],params=params,
                                labels_unchanged=True))
            if i<4 and variant==0:
                cv2.imwrite(str(package/'training'/f'shadow-example-{i:02d}.jpg'),np.hstack([image,changed[0]]))
        if i%40==0:print('SHADOW real features',i+1,'/',len(annotation),flush=True)
    real_x=np.stack(real_x);real_aug=np.stack(real_aug)
    real_y=np.stack(real_y);real_sem=np.stack(real_sem);real_sources=np.asarray(real_sources)
    synth_x,synth_aug,synth_y,synth_rows=[],[],[],[]
    for first in range(0,512,4):
        images=[cv2.imread(str(args.synthetic_road/f'{i:03d}-rgb.png')) for i in range(first,first+4)]
        metadata=[json.loads((args.synthetic_road/f'{i:03d}.json').read_text()) for i in range(first,first+4)]
        split='train' if metadata[0]['pose_group']%5 else 'development'
        changed,params=augment_views(images,seed=args.seed+(11000 if split=='train' else 21000),
                                     key=f"road-sim-{metadata[0]['pose_group']}")
        synth_x.extend(extract(images));synth_aug.extend(extract(changed))
        for index,row in zip(range(first,first+4),metadata):
            synth_rows.append(row)
            synth_y.append(cv2.imread(str(args.synthetic_road/f'{index:03d}-road.png'),0))
        records.append(dict(kind='synthetic',source=first//4,split=split,params=params,labels_unchanged=True))
        if first%128==0:print('SHADOW synthetic features',first+4,'/512',flush=True)
    synth_x=np.stack(synth_x);synth_aug=np.stack(synth_aug);synth_y=np.stack(synth_y)
    frame_map={r['id']:r for r in read_rows(args.data/'frames.jsonl')}
    proposals=[]
    for file in sorted((args.data/'policy').glob('[A-Z]*-proposals.jsonl')):
        proposals.extend(r for r in read_rows(file) if r['split']=='train')
    by_second={}
    for r in proposals:
        frame=frame_map[r['id']]
        by_second.setdefault((frame['recording'],int(frame['source_time_sec'])),r)
    chosen=list(by_second.values())
    chosen=[chosen[i] for i in np.linspace(0,len(chosen)-1,64,dtype=int)]
    weak_x,weak_aug,weak_y=[],[],[]
    for n,proposal in enumerate(chosen):
        row=frame_map[proposal['id']]
        images=list(four_views(args.data,row['views_atlas']))
        changed,params=augment_views(images,seed=args.seed+12000,key=row['id'])
        weak_x.extend(extract(images));weak_aug.extend(extract(changed))
        with np.load(args.data/proposal['proposal_mask_archive']) as stored:
            classes=stored['classes'][proposal['proposal_mask_index']]
        weak_y.extend([cv2.resize((m==3).astype(np.uint8),(480,270),interpolation=cv2.INTER_NEAREST) for m in classes])
        records.append(dict(kind='weak',source=row['id'],split='train',params=params,labels_unchanged=True))
        if n%16==0:print('SHADOW weak features',n+1,'/64',flush=True)
    weak_x=np.stack(weak_x);weak_aug=np.stack(weak_aug);weak_y=np.stack(weak_y)
    if cache:
        extract.finish()
        assert records==cached_records, 'The RGB augmentation parameters differ from the feature cache'
        del extract
    save(package/'training/shadow-augmentation.json',dict(schema=SCHEMA,variants=records,
        input_augmentation_before_flyvis=True,inference_preprocessing_changed=False,
        geometry_and_labels_unchanged=True,seed_namespaces_disjoint=True))
    # Keep a verifiable cache of newly extracted activities, not a transform of old activity tensors.
    if not cache:
        np.savez_compressed(package/'training/shadow-feature-cache.npz',real=real_x.astype(np.float16),
            real_shadow=real_aug.astype(np.float16),real_shadow_sources=real_sources,
            synthetic=synth_x.astype(np.float16),synthetic_shadow=synth_aug.astype(np.float16),
            weak=weak_x.astype(np.float16),weak_shadow=weak_aug.astype(np.float16),
            encoder_sha256=encoder.details['bundle_sha256'],
            annotation_sha256=digest(args.data/'road/annotations.jsonl'))
    device=torch.device(device)
    original=[torch.as_tensor(v,device=device) for v in [real_x,synth_x,weak_x]]
    augmented=[torch.as_tensor(v,device=device) for v in [real_aug,synth_aug,weak_aug]]
    targets=[real_y,synth_y,weak_y]
    weights=[pixel_weights(v,device) for v in targets]
    grass=np.stack([cv2.resize(((sem==11)&(target==0)).astype(np.float32),(176,96),
                             interpolation=cv2.INTER_AREA) for sem,target in zip(real_sem,real_y)])
    weights[0]=(weights[0][0],weights[0][1]+5*torch.as_tensor(grass,device=device)[:,None])
    fit=[i for i,r in enumerate(rows) if r['split']=='train']
    dev=[i for i,r in enumerate(rows) if r['split']=='development']
    held=[i for i,r in enumerate(rows) if r['split']=='validation']
    aug_by_source=defaultdict(list)
    for i,source in enumerate(real_sources):aug_by_source[int(source)].append(i)
    aug_dev=[i for i,s in enumerate(real_sources) if rows[int(s)]['split']=='development']
    aug_held=[i for i,s in enumerate(real_sources) if rows[int(s)]['split']=='validation']
    sf=[i for i,r in enumerate(synth_rows) if r['pose_group']%5]
    sd=[i for i,r in enumerate(synth_rows) if not r['pose_group']%5]
    shadow_rows=[dict(rows[int(i)]) for i in real_sources]
    shadow_y=real_y[real_sources];shadow_sem=real_sem[real_sources]
    parent=torch.load(args.parent/'road/road-readout.pt',map_location='cpu',weights_only=True)
    model=RoadReadout().to(device);model.load_state_dict(parent['state_dict'])
    threshold=float(parent['threshold'])
    before=evaluate(model,original[0],real_y,real_sem,held,rows,threshold)
    before_shadow=evaluate(model,augmented[0],shadow_y,shadow_sem,aug_held,shadow_rows,threshold)
    rng=np.random.default_rng(args.seed)
    optimizer=torch.optim.AdamW(model.parameters(),lr=.00010,weight_decay=.0001)
    best=-1.;selected=0;history=[]

    @torch.inference_mode()
    def synthetic_score():
        model.eval();matrices=[]
        for tx in [original[1],augmented[1]]:
            tp=fp=fn=0
            for start in range(0,len(sd),8):
                ids=sd[start:start+8]
                logits=F.interpolate(model(tx[ids]),(270,480),mode='bilinear',align_corners=False)
                pred=logits[:,0].sigmoid().cpu().numpy()>=threshold
                truth=synth_y[ids]!=0
                tp+=int((pred&truth).sum());fp+=int((pred&~truth).sum());fn+=int((~pred&truth).sum())
            matrices.append(tp/max(1,tp+fp+fn))
        return float(np.mean(matrices))

    def supervised(logits,pos,neg):
        return (.60*loss_for(logits[:4],pos[:4],neg[:4])
                +.40*loss_for(logits[4:12],pos[4:12],neg[4:12]))

    for epoch in range(1,args.road_epochs+1):
        model.train();losses=[]
        for step in range(max(1,len(sf)//8)):
            ids=[rng.choice(fit,4).tolist(),rng.choice(sf,8).tolist(),rng.choice(len(weak_x),4).tolist()]
            aug_ids=[[int(rng.choice(aug_by_source[i])) for i in ids[0]],ids[1],ids[2]]
            x=torch.cat([v[ix] for v,ix in zip(original[:2],ids[:2])])
            xa=torch.cat([v[ix] for v,ix in zip(augmented[:2],aug_ids[:2])])
            pos=torch.cat([w[0][ix] for w,ix in zip(weights[:2],ids[:2])])
            neg=torch.cat([w[1][ix] for w,ix in zip(weights[:2],ids[:2])])
            if rng.random()<.5:x,xa,pos,neg=[v.flip(-1) for v in [x,xa,pos,neg]]
            logits=F.interpolate(model(torch.cat([x,xa])),(96,176),mode='bilinear',align_corners=False)
            original_logits,shadow_logits=logits[:12],logits[12:]
            known=(pos+neg).clamp(0,1)
            consistency=(((original_logits.sigmoid()-shadow_logits.sigmoid())**2)*known).sum()/known.sum().clamp_min(1.)
            loss=.5*supervised(original_logits,pos,neg)+.5*supervised(shadow_logits,pos,neg)+.10*consistency
            optimizer.zero_grad(set_to_none=True);loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(),1.);optimizer.step();losses.append(float(loss.detach()))
        raw=evaluate(model,original[0],real_y,real_sem,dev,rows,threshold)
        shade=evaluate(model,augmented[0],shadow_y,shadow_sem,aug_dev,shadow_rows,threshold)
        synthetic=synthetic_score()
        score=(.35*raw['by_group']['all']['balanced_iou']+.35*shade['by_group']['all']['balanced_iou']+.30*synthetic
               -.15*(raw['vegetation_false_positive_rate']+shade['vegetation_false_positive_rate'])/2)
        metric=dict(epoch=epoch,loss=float(np.mean(losses)),development_raw=raw['by_group']['all'],
                    development_shadow=shade['by_group']['all'],synthetic_iou=synthetic,selection_score=score,
                    development_vegetation_fp=raw['vegetation_false_positive_rate'],
                    development_shadow_vegetation_fp=shade['vegetation_false_positive_rate'])
        history.append(metric);print('SHADOW_ROAD',json.dumps(metric),flush=True)
        if score>best:
            best,selected=score,epoch
            state={k:v.detach().cpu().clone() for k,v in model.state_dict().items()}
        if epoch-selected>=12:break
    model.load_state_dict(state)
    after=evaluate(model,original[0],real_y,real_sem,held,rows,threshold)
    after_shadow=evaluate(model,augmented[0],shadow_y,shadow_sem,aug_held,shadow_rows,threshold)
    if cache:
        fresh=CircuitFeatures(encoder)
        raw_features,shadow_features=[],[]
        for j,i in enumerate(held):
            row=rows[i];image=four_views(args.data,row['views_atlas'])[row['view_index']]
            changed,_=augment_views([image],seed=args.seed+30000,key=row['views_atlas'],
                                     variant=0,view_indices=[row['view_index']])
            raw_features.append(fresh([image])[0]);shadow_features.append(fresh(changed)[0])
            if j%40==0:print('Exact FP32 reference evaluation',j+1,'/',len(held),flush=True)
        tx=torch.as_tensor(np.stack(raw_features),device=device)
        ta=torch.as_tensor(np.stack(shadow_features),device=device)
        ys=real_y[held];ss=real_sem[held];rs=[rows[i] for i in held];ix=list(range(len(held)))
        model.load_state_dict(parent['state_dict'])
        before=evaluate(model,tx,ys,ss,ix,rs,threshold)
        before_shadow=evaluate(model,ta,ys,ss,ix,rs,threshold)
        model.load_state_dict(state)
        after=evaluate(model,tx,ys,ss,ix,rs,threshold)
        after_shadow=evaluate(model,ta,ys,ss,ix,rs,threshold)
    folder=package/'road';folder.mkdir()
    torch.save(dict(schema=parent['schema'],state_dict=state,threshold=threshold,
                    encoder_sha256=parent['encoder_sha256']),folder/'road-readout.pt')
    provenance=dict(schema='flyvis.road_readout.real360.shadow.v1',data=str(args.data),
        dataset_manifest_sha256=digest(args.data/'frames.jsonl'),annotation_manifest_sha256=digest(args.data/'road/annotations.jsonl'),
        encoder_frozen=True,encoder_sha256=parent['encoder_sha256'],parent_sha256=digest(args.parent/'road/road-readout.pt'),
        weights_sha256=digest(folder/'road-readout.pt'),profile_sha256=profile_sha256(load_profile()),
        seed=args.seed,selected_epoch=selected,threshold=threshold,fit_manual_views=len(fit),development_manual_views=len(dev),
        heldout_manual_views=len(held),fit_synthetic_views=len(sf),heldout_synthetic_views=len(sd),weak_real_frames=len(chosen),
        manual_weight=.60,synthetic_weight=.40,real_pseudo_weight=0.,consistency_weight=.10,
        vegetation_negative_weight_multiplier=6.,weak_teacher_disabled=True,
        reused_feature_cache_sha256=digest(cache) if cache else None,
        training_feature_cache_precision='float16 replay to float32' if cache else 'fresh float32',
        reference_evaluation_precision='fresh float32',
        shadow_augmentation=SCHEMA,inference_preprocessing_changed=False,
        fit_augmented_manual_views=sum(rows[int(i)]['split']=='train' for i in real_sources),
        evaluation=dict(before=before,after=after,before_synthetic_shadow=before_shadow,after_synthetic_shadow=after_shadow),
        history=history,wall_sec=time.monotonic()-started)
    save(folder/'simulation-training.json',provenance)
    save(package/'training/road-splits.json',dict(real=[{k:r[k] for k in ['id','recording','source_time_sec','split','view']} for r in rows],
        weak_frames=[r['id'] for r in chosen],synthetic_train=sf,synthetic_validation=sd,
        variants_preserve_base_partition=True))
    print('SHADOW_ROAD_COMPLETE',selected,after['by_group']['all']['road_iou'],after_shadow['by_group']['all']['road_iou'],flush=True)

"""Train compatible Flyvis road and camera-mask action readouts on the prepared real360 dataset."""
import argparse
from hashlib import sha256
import json
from pathlib import Path
import shutil
import time

import cv2
import numpy as np
import torch
from torch.nn import functional as F

from connectome360_dataset import four_views, read_rows
from connectome_flow import FlyvisFlow
from connectome_road import CircuitFeatures, ConnectomeRoadPerception, RoadReadout
from connectome_policy import DriveReadout, policy_pixels
from dangjin_sensor_faults import SensorFaults
from finetune_connectome_road import evaluate, loss_for, pixel_weights
from train_connectome_policy import teacher
from surround_camera import load_profile, profile_sha256


def digest(path):
    return sha256(Path(path).read_bytes()).hexdigest()


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False) + '\n')


def take_seconds(rows):
    chosen = {}
    for row in rows:
        key = row['recording'], int(row['source_time_sec'])
        chosen.setdefault(key, row)
    return list(chosen.values())


def pick(rows, count):
    return [rows[i] for i in np.linspace(0, len(rows)-1, min(count, len(rows)), dtype=int)]


def road_training(args, encoder, package, device):
    started = time.monotonic()
    annotation = read_rows(args.data/'road/annotations.jsonl')
    extract = CircuitFeatures(encoder)
    features, labels, semantic, rows = [], [], [], []
    for i, row in enumerate(annotation):
        views = four_views(args.data, row['views_atlas'])
        features.append(extract([views[row['view_index']]])[0])
        labels.append(cv2.imread(str(args.data/row['road_label']), 0))
        semantic.append(cv2.imread(str(args.data/row['semantic_label']), 0))
        rows.append(dict(row, clip=row['recording']))
        if i % 40 == 0:
            print('Real partial annotations', i+1, '/', len(annotation), flush=True)
    real_x = np.stack(features)
    real_y, real_sem = np.stack(labels), np.stack(semantic)
    synth_rows, synth_x, synth_y = [], [], []
    for first in range(0, 512, 4):
        paths = [args.synthetic_road/f'{i:03}-rgb.png' for i in range(first, first+4)]
        synth_x.extend(extract([cv2.imread(str(p)) for p in paths]))
        for i in range(first, first+4):
            row = json.loads((args.synthetic_road/f'{i:03}.json').read_text())
            synth_rows.append(row)
            synth_y.append(cv2.imread(str(args.synthetic_road/f'{i:03}-road.png'), 0))
        if first % 128 == 0:
            print('Current-scene road features', first+4, '/ 512', flush=True)
    synth_x, synth_y = np.stack(synth_x), np.stack(synth_y)
    frame_map = {r['id']: r for r in read_rows(args.data/'frames.jsonl')}
    train_proposals = []
    for file in sorted((args.data/'policy').glob('[A-Z]*-proposals.jsonl')):
        train_proposals.extend(r for r in read_rows(file) if r['split']=='train')
    weak = pick(take_seconds([dict(frame_map[r['id']], proposal=r) for r in train_proposals]), 64)
    weak_x, weak_y = [], []
    for row in weak:
        views = four_views(args.data, row['views_atlas'])
        weak_x.extend(extract(list(views)))
        p = row['proposal']
        with np.load(args.data/p['proposal_mask_archive']) as archive:
            classes = archive['classes'][p['proposal_mask_index']]
        weak_y.extend([cv2.resize((m==3).astype(np.uint8), (480,270), interpolation=cv2.INTER_NEAREST)
                       for m in classes])
    weak_x, weak_y = np.stack(weak_x), np.stack(weak_y)
    tensors = [torch.as_tensor(x, device=device) for x in (real_x, synth_x, weak_x)]
    weights = [pixel_weights(y, device) for y in (real_y, synth_y, weak_y)]
    fit = [i for i, r in enumerate(rows) if r['split']=='train']
    dev = [i for i, r in enumerate(rows) if r['split']=='development']
    held = [i for i, r in enumerate(rows) if r['split']=='validation']
    sf = [i for i, r in enumerate(synth_rows) if r['pose_group'] % 5]
    sd = [i for i, r in enumerate(synth_rows) if not r['pose_group'] % 5]
    parent = torch.load(args.parent/'road/road-readout.pt', map_location='cpu', weights_only=True)
    model = RoadReadout().to(device)
    model.load_state_dict(parent['state_dict'])
    threshold = float(parent['threshold'])
    rng = np.random.default_rng(args.seed)
    opt = torch.optim.AdamW(model.parameters(), lr=.00015, weight_decay=.0001)
    folder = package/'road'
    folder.mkdir()
    history = []
    best = -1.
    selected = 0

    @torch.inference_mode()
    def synthetic_score():
        model.eval()
        tp=fp=fn=0
        for start in range(0, len(sd), 8):
            ids = sd[start:start+8]
            logits = F.interpolate(model(tensors[1][ids]), (270,480), mode='bilinear', align_corners=False)
            pred = logits[:,0].sigmoid().cpu().numpy() >= threshold
            gt = synth_y[ids]!=0
            tp += int((pred & gt).sum())
            fp += int((pred & ~gt).sum())
            fn += int((~pred & gt).sum())
        return tp/max(1,tp+fp+fn)

    for epoch in range(1, args.road_epochs+1):
        model.train()
        losses=[]
        for start in range(0, len(sf), 8):
            ids=[rng.choice(fit,4).tolist(),rng.choice(sf,8).tolist(),rng.choice(len(weak_x),4).tolist()]
            x=torch.cat([tx[ix] for tx,ix in zip(tensors,ids)])
            pos=torch.cat([w[0][ix] for w,ix in zip(weights,ids)])
            neg=torch.cat([w[1][ix] for w,ix in zip(weights,ids)])
            if rng.random()<.5:
                x,pos,neg=(v.flip(-1) for v in (x,pos,neg))
            logits=F.interpolate(model(x),(96,176),mode='bilinear',align_corners=False)
            loss=(.45*loss_for(logits[:4],pos[:4],neg[:4])
                  +.45*loss_for(logits[4:12],pos[4:12],neg[4:12])
                  +.10*loss_for(logits[12:],pos[12:],neg[12:]))
            opt.zero_grad(set_to_none=True)
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(),1.)
            opt.step()
            losses.append(loss.item())
        real = evaluate(model,tensors[0],real_y,real_sem,dev,rows,threshold)['by_group']['all']
        synth_iou = synthetic_score()
        score = .5*real['balanced_iou']+.5*synth_iou
        metric=dict(epoch=epoch,loss=float(np.mean(losses)),development_balanced_iou=real['balanced_iou'],
                    synthetic_heldout_iou=synth_iou,selection_score=score)
        history.append(metric)
        print('ROAD',json.dumps(metric),flush=True)
        if score>best:
            best,selected=score,epoch
            state={k:v.detach().cpu().clone() for k,v in model.state_dict().items()}
        if epoch-selected>=10:
            break
    model.load_state_dict(parent['state_dict'])
    before=evaluate(model,tensors[0],real_y,real_sem,held,rows,threshold)
    model.load_state_dict(state)
    after=evaluate(model,tensors[0],real_y,real_sem,held,rows,threshold)
    torch.save(dict(schema=parent['schema'],state_dict=state,threshold=threshold,
                    encoder_sha256=parent['encoder_sha256']),folder/'road-readout.pt')
    provenance=dict(schema='flyvis.road_readout.real360.v2',data=str(args.data),
                    dataset_manifest_sha256=digest(args.data/'frames.jsonl'),
                    annotation_manifest_sha256=digest(args.data/'road/annotations.jsonl'),
                    encoder_frozen=True,encoder_sha256=parent['encoder_sha256'],
                    parent_sha256=digest(args.parent/'road/road-readout.pt'),
                    weights_sha256=digest(folder/'road-readout.pt'),profile_sha256=profile_sha256(load_profile()),
                    seed=args.seed,selected_epoch=selected,threshold=threshold,
                    fit_manual_views=len(fit),development_manual_views=len(dev),heldout_manual_views=len(held),
                    fit_synthetic_views=len(sf),heldout_synthetic_views=len(sd),weak_real_frames=len(weak),
                    synthetic_directory=str(args.synthetic_road),
                    real_pseudo_weight=.1,manual_weight=.45,synthetic_weight=.45,
                    evaluation=dict(before=before,after=after),history=history,wall_sec=time.monotonic()-started)
    write_json(folder/'simulation-training.json',provenance)
    write_json(package/'training/road-splits.json',
               dict(real=[{k:r[k] for k in ('id','recording','source_time_sec','split','view')} for r in rows],
                    weak_frames=[r['id'] for r in weak],
                    synthetic_train=sf,synthetic_validation=sd))
    print('ROAD complete',before['by_group']['all']['road_iou'],after['by_group']['all']['road_iou'],flush=True)


def policy_training(args, encoder, package, device):
    started=time.monotonic()
    road=ConnectomeRoadPerception(encoder,package/'road',device)
    poses=[json.loads(p.read_text()) for p in sorted(args.synthetic_policy.glob('[0-9][0-9][0-9][0-9].json'))]
    if len(poses)!=768:
        raise ValueError('Expected 768 current-scene policy poses')
    render_provenance=json.loads(args.synthetic_policy_provenance.read_text())
    for name,expected in render_provenance['render_source_sha256'].items():
        if digest(Path('environments/dangjin')/name)!=expected:
            raise ValueError('Cached synthetic actor data differs from current renderer')
    route=np.asarray(json.loads(Path('environments/dangjin/world.json').read_text())['perimeter_route'])
    xs,ys,parts,origins,ids,packed_masks=[],[],[],[],[],[]
    illumination_variants=[]
    shadow_enabled=getattr(args,'shadow_augment',False)
    shadow_records=[]
    def add_input(frames,target,partition,origin,sample_id):
        variants=[('original',frames)]
        if shadow_enabled:
            from connectome_shadow import augment_views
            changed,params=augment_views(frames,seed=args.seed+(51000 if partition=='train' else 61000),key=sample_id)
            variants.append(('shadow',changed))
            shadow_records.append(dict(id=sample_id,split=partition,params=params,command_target_unchanged=True))
        for illumination,images in variants:
            mask,_=road.predict_views(images)
            packed_masks.append(np.packbits(mask.reshape(-1)))
            xs.append(policy_pixels(mask));ys.append(target)
            parts.append(partition);origins.append(origin)
            ids.append(sample_id if illumination=='original' else sample_id+'|shadow')
            illumination_variants.append(illumination)
    for i,row in enumerate(poses):
        frames=[cv2.imread(str(args.synthetic_policy/f'{row["index"]:04d}-{sector}-rgb.png')) for sector in range(4)]
        add_input(frames,[*teacher(row,route,max_speed=args.max_speed),0.],
                  'train' if row['pose_group']%5 else 'development','synthetic',f'sim-{i:04d}')
        if i%96==0:
            print('Policy synthetic inference',i+1,'/ 768',flush=True)
    frame_map={r['id']:r for r in read_rows(args.data/'frames.jsonl')}
    proposed=[]
    for file in sorted((args.data/'policy').glob('[A-Z]*-proposals.jsonl')):
        proposed.extend(r for r in read_rows(file) if r['split'] in ('train','development'))
    selected=take_seconds([dict(frame_map[r['id']],proposal=r) for r in proposed])
    for i,row in enumerate(selected):
        cmd=row['proposal']['estimated_command']
        add_input(list(four_views(args.data,row['views_atlas'])),
                  [cmd['vx_mps']/args.max_speed,cmd['yaw_radps']/.35,float(cmd['stop'])],
                  row['split'],'real_estimated',row['id'])
        if i%128==0:
            print('Policy real inference',i+1,'/',len(selected),flush=True)
    x=np.asarray(xs,np.float32)
    y=np.asarray(ys,np.float32)
    packed_masks=np.stack(packed_masks)
    del xs,ys,road
    np.savez_compressed(package/'training/policy-inputs.npz',pixels=x,targets=y,ids=np.asarray(ids),
                        partitions=np.asarray(parts),origins=np.asarray(origins),
                        illumination_variants=np.asarray(illumination_variants),
                        native_masks_packed=packed_masks,native_mask_shape=np.array([4,270,480]),
                        road_sha256=digest(package/'road/road-readout.pt'))
    fit={source:np.array([i for i,p in enumerate(parts) if p=='train' and origins[i]==source])
         for source in ('synthetic','real_estimated')}
    valid={source:np.array([i for i,p in enumerate(parts) if p=='development' and origins[i]==source])
           for source in fit}
    torch.set_default_device('cpu')
    model=DriveReadout().to(device)
    parent=torch.load(args.init_policy/'drive-readout.pt',map_location='cpu',weights_only=True)
    model.load_state_dict(parent['state_dict'])
    optimizer=torch.optim.AdamW(model.parameters(),lr=.0003,weight_decay=.0001)
    rng=np.random.default_rng(args.seed+1)
    def batch(indices, random, seed_low):
        values=x[indices].copy()
        targets=y[indices,:2].copy()
        stops=y[indices,2].astype(np.int64).copy()
        for j in range(len(values)):
            r=random.random()
            if r<.16:
                values[j]=0
                stops[j]=1
                targets[j]=0
            elif r<.8:
                noise=SensorFaults(dict(enabled=True,profile='severe' if r<.48 else 'normal',
                                       seed=int(random.integers(seed_low,seed_low+1000))))
                event=noise.active_event('holes',3.)
                if event is None:
                    event=noise.events['holes'][0]
                native=np.unpackbits(packed_masks[indices[j]],count=4*270*480).reshape(4,270,480)
                damaged,_=noise.apply(native,(event['start']+event['end'])/2)
                values[j]=policy_pixels(damaged)
                if not values[j].any():
                    stops[j]=1
                    targets[j]=0
        mirror=random.random(len(values))<.5
        values[mirror]=values[mirror][:,[0,3,2,1],:,:,][:,:,:,::-1]
        targets[mirror,1]*=-1
        return [torch.as_tensor(v.copy(),device=device) for v in (values,targets,stops)]

    @torch.inference_mode()
    def score(source):
        model.eval()
        predictions=[];truth=[];flags=[];logits_all=[]
        local=np.random.default_rng(args.seed+2047)
        for j in range(0,len(valid[source]),64):
            ids2=valid[source][j:j+64]
            value,target,stop=batch(ids2,local,2000)
            pred,logit=model(value)
            predictions.append(pred.cpu().numpy())
            truth.append(target.cpu().numpy())
            flags.append(stop.cpu().numpy())
            logits_all.append(logit.argmax(1).cpu().numpy())
        pred,truth,z,zz=map(np.concatenate,(predictions,truth,flags,logits_all))
        moving=z==0
        err=np.abs(pred[moving]-truth[moving]).mean(0)
        return dict(speed_mae_mps=float(err[0]*args.max_speed),yaw_mae_radps=float(err[1]*.35),
                    stop_accuracy=float((zz==z).mean()),
                    stop_recall=float(((zz==1)&(z==1)).sum()/max(1,(z==1).sum())))
    best=float('inf');best_epoch=0;history=[]
    for epoch in range(1,args.policy_epochs+1):
        losses=[]
        for step in range(max(1,len(fit['synthetic'])//48)):
            indices=np.r_[rng.choice(fit['synthetic'],48),rng.choice(fit['real_estimated'],16)]
            value,target,stop=batch(indices,rng,1000)
            model.train()
            pred,logit=model(value)
            move=(stop==0)[:,None]
            terms=(((pred-target)**2)*torch.tensor([1.,5.],device=device)*move).mean(1)
            terms+=F.cross_entropy(logit,stop,reduction='none')*.25
            loss=.8*terms[:48].mean()+.2*terms[48:].mean()
            optimizer.zero_grad(set_to_none=True)
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(),2.)
            optimizer.step()
            losses.append(loss.item())
        sim_score,real_score=score('synthetic'),score('real_estimated')
        value=(.8*(sim_score['yaw_mae_radps']+.1*sim_score['speed_mae_mps']+1-sim_score['stop_accuracy'])
               +.2*(real_score['yaw_mae_radps']+.1*real_score['speed_mae_mps']+1-real_score['stop_accuracy']))
        metric=dict(epoch=epoch,loss=float(np.mean(losses)),synthetic_validation=sim_score,
                    real_estimate_agreement=real_score,selection_score=value)
        history.append(metric)
        if epoch%5==0:
            print('POLICY',json.dumps(metric),flush=True)
        if value<best:
            best,best_epoch=value,epoch
            state={k:v.detach().cpu().clone() for k,v in model.state_dict().items()}
        if epoch-best_epoch>=25:
            break
    folder=package/'policy'
    folder.mkdir()
    torch.save(dict(schema=parent['schema'],state_dict=state,road_sha256=digest(package/'road/road-readout.pt')),
               folder/'drive-readout.pt')
    world=json.loads(Path('environments/dangjin/world.json').read_text())
    training=dict(schema='dangjin.drive_training.vision.v2',data=str(args.data),
                  dataset_manifest_sha256=digest(args.data/'frames.jsonl'),
                  encoder_sha256=encoder.details['bundle_sha256'],encoder_frozen=True,road_readout_frozen=True,
                  road_sha256=digest(package/'road/road-readout.pt'),weights_sha256=digest(folder/'drive-readout.pt'),
                  input='four camera-derived Connectome masks -> policy_pixels; no route/pose/goal/FCN at runtime',
                  actor_input_shape=[4,32,64],layout_revision=world['layout_revision'],
                  camera_profile_sha256=profile_sha256(load_profile()),
                  parent_policy_sha256=digest(args.init_policy/'drive-readout.pt'),
                  synthetic_policy_directory=str(args.synthetic_policy),
                  fit_synthetic=int(len(fit['synthetic'])),development_synthetic=int(len(valid['synthetic'])),
                  fit_real_estimated=int(len(fit['real_estimated'])),development_real_estimated=int(len(valid['real_estimated'])),
                  real_sampling='first actual frame in each occupied 1-second interval',
                  max_speed_mps=args.max_speed,
                  shadow_augmentation_enabled=shadow_enabled,
                  illumination_variants=['original','shadow'] if shadow_enabled else ['original'],
                  temporal_memory=False,
                  real_targets_preserve_original_mps=True,
                  real_estimated_weight=.2,synthetic_teacher_weight=.8,executed_real_commands=False,
                  real_targets='unreviewed offline FCN/geometry pseudo commands; explicitly enabled by this training run',
                  synthetic_targets='offline pure-pursuit of synthetic route; never called at runtime',
                  train_fault_seed_range=[1000,1999],validation_fault_seed_range=[2000,2999],
                  selected_epoch=history[best_epoch-1],history=history,seed=args.seed,wall_sec=time.monotonic()-started)
    write_json(folder/'training.json',training)
    if shadow_enabled:
        write_json(package/'training/policy-shadow-augmentation.json',dict(variants=shadow_records,
                   image_augmentation_before_detector=True,command_targets_unchanged=True))
    write_json(package/'training/policy-splits.json',dict(ids=ids,parts=parts,origins=origins))
    print('POLICY complete',best_epoch,flush=True)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--data',type=Path,required=True)
    p.add_argument('--parent',type=Path,required=True)
    p.add_argument('--synthetic-road',type=Path,required=True)
    p.add_argument('--synthetic-policy',type=Path,required=True)
    p.add_argument('--synthetic-policy-provenance',type=Path,required=True)
    p.add_argument('--init-policy',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--allow-estimated-actions',action='store_true')
    p.add_argument('--shadow-augment',action='store_true',help='Train with coherent 360 shadow/exposure variants before Flyvis feature extraction')
    p.add_argument('--shadow-feature-cache',type=Path,help='Reuse activities previously extracted from the exact same seeded shadow images')
    p.add_argument('--stage',choices=['all','policy'],default='all')
    p.add_argument('--max-speed',type=float,default=2.,help='simulation teacher/actor speed scale; not a robot deployment limit')
    p.add_argument('--road-epochs',type=int,default=48)
    p.add_argument('--policy-epochs',type=int,default=120)
    p.add_argument('--seed',type=int,default=62417)
    a=p.parse_args()
    if not .1 <= a.max_speed <= 20.:
        p.error('--max-speed must be between 0.1 and 20.0 m/s (simulation only)')
    if a.road_epochs<1 or a.policy_epochs<1:
        p.error('Both epoch limits must be positive')
    if not a.allow_estimated_actions:
        p.error('This dataset has unreviewed real-action targets. Explicit --allow-estimated-actions is required.')
    if not json.loads((a.data/'verification.json').read_text())['passed']:
        raise ValueError('Dataset integrity checks did not pass')
    a.output.mkdir(parents=True,exist_ok=True)
    (a.output/'training').mkdir(exist_ok=True)
    if (a.stage=='all' and (a.output/'road').exists()) or (a.output/'policy').exists():
        raise ValueError('Choose a fresh model package')
    torch.set_num_threads(2)
    cv2.setNumThreads(2)
    torch.manual_seed(a.seed)
    device='mps' if torch.backends.mps.is_available() else 'cpu'
    expected=json.loads((a.parent/'road/simulation-training.json').read_text())['encoder_sha256']
    bundle=a.parent/'flyvis_encoder.pt'
    shutil.copyfile(bundle,a.output/'flyvis_encoder.pt')
    encoder=FlyvisFlow(bundle,expected,'cpu')
    if a.stage=='all':
        if a.shadow_augment:
            from train_connectome360_shadow import road_training_shadow
            road_training_shadow(a,encoder,a.output,device)
        else:
            road_training(a,encoder,a.output,device)
    policy_training(a,encoder,a.output,device)
    write_json(a.output/'model.json',dict(schema='connectome360.camera_only.pair.v1',data=str(a.data),
               road_sha256=digest(a.output/'road/road-readout.pt'),
               policy_sha256=digest(a.output/'policy/drive-readout.pt'),
               encoder_sha256=expected,encoder_frozen=True,simulation_only=True))
    shutil.copyfile(__file__,a.output/'training/train_connectome360.py')


if __name__=='__main__':
    main()

#!/usr/bin/env python3
"""Generate explicitly unverified offline teacher labels for connectome training.

The segmentation teacher is NOT a replacement for the intended connectome model.
Missing depth, uncertain surfaces and ambiguous paths produce abstentions, not
invented expert commands. No ROS or network control output is implemented.
"""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import time

import cv2
import numpy as np
import torch
from transformers import AutoImageProcessor, MaskFormerForInstanceSegmentation

from build_connectum_dataset import dump,line,source_ids
from label_road_sample_dataset import ground_plane,unproject_ground,corridor_evidence,project_points
from r50_checkpoint import validate_checkpoint

VERSION='connectum.front.pseudo.shards.1hz.v1'
ROAD_IDS=[13,23,24,41]
DYNAMIC_IDS=[1,19,20,21,22,52,54,55,56,57,59,60,61,62]
SETTINGS={'schema':VERSION,'teacher_only':True,'human_verified':False,'label_size_wh':[640,360],
    'target_label_hz':1,'context_image_hz':5,'interpolate_missing_labels':False,
    'storage':'NPZ shards, 64 frames per shard; uint8 arrays semantic/road/confidence/lidar_ground',
    'road_surface_ids':ROAD_IDS,'unknown_value':255,'minimum_relative_score':.60,'minimum_relative_margin':.15,
    'straight_speed_mps':.25,'turn_speed_mps':.18,'max_yaw_rate_radps':.18,
    'target_image_y_fraction':.60,'near_obstacle_stop_m':1.5,'dynamic_yield_m':4.,
    'command_units':{'linear_x':'m/s','angular_z':'rad/s'},'positive_yaw':'left',
    'coordinate_frame':'assumed base_link: x forward, y left, z up',
    'confidence_is_calibrated':False,'policy_values_are_design_assumptions':True,
    'model_use':'offline pseudo-label generation only; deployment architecture remains front-camera connectome',
    'forbidden_inferences':['paved surface implies site permission','no lidar returns implies free space','predicted command equals recorded command']}


def write_png(path,a):
    ok,data=cv2.imencode('.png',a,[cv2.IMWRITE_PNG_COMPRESSION,2])
    if not ok:raise RuntimeError('Could not encode label')
    payload=data.tobytes();path.write_bytes(payload)
    return hashlib.sha256(payload).hexdigest()


def surface_target(mask):
    """Visible connected surface proposal. No gap filling or scene map access."""
    binary=(mask==1).astype(np.uint8)
    count,components,stats,_=cv2.connectedComponentsWithStats(binary,8)
    h,w=mask.shape
    seed=components[int(.75*h):int(.93*h),int(.35*w):int(.65*w)]
    ids,n=np.unique(seed[seed>0],return_counts=True)
    if not len(ids):return None,[],{'status':'no_near_surface_component'}
    component=int(ids[np.argmax(n)])
    if stats[component,cv2.CC_STAT_AREA]<.025*w*h:return None,[],{'status':'surface_component_too_small'}
    points=[];widths=[];ambiguous=False;center=w/2
    for y in [int(.86*h),int(.75*h),int(.66*h),int(.60*h)]:
        selected=components[y]==component
        edges=np.diff(np.r_[False,selected,False].astype(np.int8));starts=np.flatnonzero(edges==1);ends=np.flatnonzero(edges==-1)
        runs=[(int(a),int(b)) for a,b in zip(starts,ends) if b-a>=max(12,.04*w)]
        if not runs:return None,points,{'status':'surface_gap_at_lookahead'}
        close=[(a,b) for a,b in runs if a-.12*w<=center<=b+.12*w]
        if len(close)!=1:return None,points,{'status':'multiple_or_disconnected_surface_choices'}
        a,b=close[0];new_center=(a+b-1)/2
        if abs(new_center-center)>.18*w:ambiguous=True
        center=new_center;points.append([float(center),float(y)]);widths.append(b-a)
    if ambiguous:return None,points,{'status':'abrupt_center_change_requires_review','widths_pixels':widths}
    return points[-1],points,{'status':'candidate','widths_pixels':widths,
        'limitation':'midpoint of visible connected surface; no global route or forbidden-zone knowledge'}


def make_command(target,evidence,valid_sync,dynamic_close):
    if not valid_sync:return None,'camera_lidar_time_mismatch'
    if target is None or not evidence.get('valid'):return None,'geometry_unavailable'
    if evidence['ground_support_fraction']<.6:return None,'insufficient_lidar_ground_support'
    obstacle=evidence['nearest_supported_obstacle_along_path_m']
    if dynamic_close:
        return {'action':'stop','linear_x_mps':0.,'linear_y_mps':0.,'angular_z_radps':0.,'stop':True,'reason':'projected_dynamic_object_and_lidar_in_near_corridor'},'candidate'
    if obstacle is not None and obstacle<SETTINGS['near_obstacle_stop_m']:
        return {'action':'stop','linear_x_mps':0.,'linear_y_mps':0.,'angular_z_radps':0.,'stop':True,'reason':'near_supported_non_ground_returns_in_corridor'},'candidate'
    x,y=target[:2];bearing=np.degrees(np.arctan2(y,x));v=.25 if abs(bearing)<=5 else .18
    omega=float(np.clip(2*v*y/(x*x+y*y),-.18,.18))
    action='turn_left' if bearing>5 else 'turn_right' if bearing<-5 else 'forward'
    return {'action':action,'linear_x_mps':v,'linear_y_mps':0.,'angular_z_radps':round(omega,6),'stop':False,'reason':'visible_surface_center_and_local_lidar_ground'},'candidate'


def geometry(root,row,mask,semantic,relative_confidence,cal):
    target_pixel,path_pixels,visual=surface_target(mask)
    result={'is_estimated':True,'visual_path_pixels':path_pixels,'visual_path_status':visual,'working_image_size_wh':[640,360],
        'extrinsic_verified':False,'command_status':None,'estimated_command':None,'ground_plane':None,'target_base_m':None}
    sparse=np.full((360,640),255,np.uint8)
    if row['lidar'] is None:
        result['command_status']='abstain_missing_lidar';return result,sparse
    with np.load(root/row['lidar']['path']) as cloud:
        sensor=cloud['xyz_sensor_m'][cloud['valid_point_mask']]
    B=np.array(cal['base_from_lidar']);C=np.array(cal['camera_from_base'])
    points=sensor@B[:3,:3].T+B[:3,3]
    plane,fit=ground_plane(points);result['ground_plane']=fit
    if plane is None:result['command_status']='abstain_ground_fit_failed';return result,sparse
    info=json.loads((root/row['camera_info']['path']).read_text())[row['camera_info']['index']]
    K=np.array(info['K']).reshape(3,3);K[0]*=640/info['width'];K[1]*=360/info['height'];D=np.array(info['D'])
    uv,inside=project_points(points,K,D,C)
    inside&=(uv[:,0]<640)&(uv[:,1]<360)
    pixel=np.zeros((len(points),2),int);pixel[inside]=uv[inside].astype(int)
    heights=points[:,2]-(points[:,:2]@plane[:2]+plane[2]);ground=abs(heights)<=.09
    non_ground=(heights>=.18)&(heights<=2.5)
    selected=np.flatnonzero(inside&(ground|non_ground))
    # Nearest return wins at a pixel; unobserved pixels remain ignore=255.
    selected=selected[np.argsort(np.linalg.norm(points[selected],axis=1))[::-1]]
    for i in selected:sparse[pixel[i,1],pixel[i,0]]=1 if ground[i] else 0
    result['lidar_projected_ground_pixels']=int((sparse==1).sum());result['lidar_projected_non_ground_pixels']=int((sparse==0).sum())
    if target_pixel is None:result['command_status']='abstain_'+visual['status'];return result,sparse
    target=unproject_ground([target_pixel],K,D,C,plane)[0]
    evidence,_,robust,path=corridor_evidence(points,plane,target)
    result['target_base_m']=target;result['corridor']=evidence
    dynamic_close=False;dynamic_points=0
    if target is not None and len(path):
        from scipy.spatial import cKDTree
        distance,_=cKDTree(path).query(points[:,:2])
        ids=np.flatnonzero(inside&robust&(distance<.65)&(points[:,0]<SETTINGS['dynamic_yield_m']))
        if len(ids):
            labels=semantic[pixel[ids,1],pixel[ids,0]];confidence=relative_confidence[pixel[ids,1],pixel[ids,0]]
            dynamic_points=int((np.isin(labels,DYNAMIC_IDS)&(confidence>=.6)).sum());dynamic_close=dynamic_points>=5
    command,status=make_command(target,evidence,row['lidar']['association_valid'],dynamic_close)
    if command:command.update(is_estimated=True,human_verified=False,confidence='low',confidence_is_calibrated=False,
        source='R50 visual pseudo-label + assumed-extrinsic LiDAR geometry + low-speed design policy',action_horizon_s=.2,
        actuation_delay_s=None,deployable=False)
    result['estimated_command']=command;result['command_status']=status;result['dynamic_projected_near_points']=dynamic_points
    return result,sparse


def process_sequence(root,directory,model,processor,device,cal,teacher_hash):
    complete=directory/'labels_completed.json'
    if complete.exists():
        status=json.loads(complete.read_text())
        if status['schema']!=VERSION or status['teacher_hash']!=teacher_hash:raise ValueError('Pseudo-label resume mismatch')
        assert hashlib.sha256((directory/'pseudo_labels.jsonl').read_bytes()).hexdigest()==status['index_sha256']
        return status
    all_rows=[json.loads(s) for s in (directory/'samples.jsonl').read_text().splitlines()]
    rows=[];last_second=None
    for row in all_rows:
        second=int(row['camera']['header_time_ns'])//1_000_000_000
        if second!=last_second:rows.append(row);last_second=second
    output=directory/'pseudo';output.mkdir(exist_ok=True)
    (output/'shards').mkdir(exist_ok=True)
    preview_ids={rows[i]['camera']['index'] for i in np.linspace(0,len(rows)-1,min(6,len(rows))).round().astype(int)}
    start=time.monotonic();labels=[];preview=[];pending={k:[] for k in ['semantic','road','confidence','lidar_ground']}
    for offset in range(0,len(rows),4):
        batch=rows[offset:offset+4];rgb=[cv2.cvtColor(cv2.imread(str(root/r['camera']['image'])),cv2.COLOR_BGR2RGB) for r in batch]
        inputs=processor(images=rgb,return_tensors='pt').to(device)
        with torch.inference_mode():
            outputs=model(**inputs)
            class_scores=outputs.class_queries_logits.softmax(-1)[...,:-1]
            mask_scores=outputs.masks_queries_logits.sigmoid()
            scores=torch.einsum('bqc,bqhw->bchw',class_scores,mask_scores)
            scores=torch.nn.functional.interpolate(scores,size=(360,640),mode='bilinear',align_corners=False)
            relative=scores/scores.sum(1,keepdim=True).clamp_min(1e-8)
            top,indices=relative.topk(2,dim=1)
            classes=indices[:,0].cpu().numpy().astype(np.uint8);confidence=top[:,0].cpu().numpy();margin=(top[:,0]-top[:,1]).cpu().numpy()
        for j,row in enumerate(batch):
            index=row['camera']['index'];name=f'{index:06d}';semantic=classes[j];small=cv2.resize(rgb[j],(640,360))
            valid=(confidence[j]>=.6)&(margin[j]>=.15)&(small.mean(2)>8)&(~np.isin(semantic,[63,64]))
            road=np.full((360,640),255,np.uint8);road[valid]=np.isin(semantic[valid],ROAD_IDS).astype(np.uint8)
            label={'schema_version':VERSION,'id':row['id'],'sequence':row['sequence'],'session':row['session'],
                'recorded_command':None,'is_estimated':True,'human_verified':False,'teacher_manifest_sha256':teacher_hash,
                'label_size_wh':[640,360],'mask_values':{'0':'other_visible_surface_or_object','1':'road_surface_candidate','255':'ignore_unknown'},
                'semantic_model':'MaskFormer R50, offline teacher only','valid_pixel_fraction':float(valid.mean()),
                'road_candidate_fraction':float((road==1).mean()),'labels':{},'label_sha256':{},
                'label_hash_format':'SHA256 of decoded uint8 C-order bytes, 360x640',
                'training_status':'pseudo_only_requires_review','expert_command_ground_truth':False}
            geo,sparse=geometry(root,row,road,semantic,confidence[j],cal)
            label['geometry']=geo;label['estimated_command']=geo.pop('estimated_command');label['command_status']=geo['command_status']
            ordinal=offset+j
            archive=output/'shards'/f'{ordinal//64:05d}.npz'
            for kind,array in [('semantic',semantic),('road',road),('confidence',np.round(confidence[j]*255).astype(np.uint8)),('lidar_ground',sparse)]:
                label['labels'][kind]={'archive':archive.relative_to(root).as_posix(),'array':kind,'index':ordinal%64}
                label['label_sha256'][kind]=hashlib.sha256(array.tobytes()).hexdigest()
                pending[kind].append(array.copy())
            if len(pending['road'])==64 or ordinal==len(rows)-1:
                np.savez_compressed(archive,**{k:np.stack(v) for k,v in pending.items()})
                pending={k:[] for k in pending}
            labels.append(label)
            if index in preview_ids:
                overlay=small.copy();overlay[road==1]=(overlay[road==1]*.55+np.array([40,220,145])*.45).astype(np.uint8)
                for xy in geo['visual_path_pixels']:cv2.circle(overlay,tuple(np.round(xy).astype(int)),3,(255,216,60),-1)
                if len(geo['visual_path_pixels'])>1:cv2.polylines(overlay,[np.round(geo['visual_path_pixels']).astype(np.int32)],False,(255,216,60),2)
                cv2.rectangle(overlay,(0,0),(640,26),(14,24,32),-1)
                c=label['estimated_command'];caption=f"{index:06d} ESTIMATE {c['action']} v={c['linear_x_mps']:.2f} w={c['angular_z_radps']:+.3f}" if c else f"{index:06d} NO COMMAND: {label['command_status']}"
                cv2.putText(overlay,caption,(6,18),cv2.FONT_HERSHEY_SIMPLEX,.39,(245,245,245),1)
                dest=directory/'previews'/f'{name}-pseudo.jpg';cv2.imwrite(str(dest),cv2.cvtColor(overlay,cv2.COLOR_RGB2BGR))
                preview.append({'id':row['id'],'image':row['camera']['image'],'overlay':dest.relative_to(root).as_posix(),
                    'command':c,'status':label['command_status'],'road_fraction':label['road_candidate_fraction']})
        if (offset//4)%25==0:print(line({'event':'label_progress','sequence':directory.name,'done':min(offset+4,len(rows)),'total':len(rows)}),end='',flush=True)
    with (directory/'pseudo_labels.jsonl.tmp').open('w') as f:
        for label in labels:f.write(line(label))
    (directory/'pseudo_labels.jsonl.tmp').replace(directory/'pseudo_labels.jsonl')
    dump(directory/'previews'/'index.json',preview)
    status={'schema':VERSION,'teacher_hash':teacher_hash,'sequence':directory.name,'samples':len(labels),'seconds':round(time.monotonic()-start,2),
        'commands':dict(Counter(l['estimated_command']['action'] for l in labels if l['estimated_command'])),
        'abstentions':dict(Counter(l['command_status'] for l in labels if l['estimated_command'] is None)),
        'index_sha256':hashlib.sha256((directory/'pseudo_labels.jsonl').read_bytes()).hexdigest()}
    dump(complete,status);print(line({'event':'labeled',**status}),end='',flush=True)
    return status


def run(root,checkpoint,device,watch):
    validate_checkpoint(checkpoint)
    teacher_hash=hashlib.sha256((checkpoint/'checkpoint-manifest.json').read_bytes()).hexdigest()
    dump(root/'pseudo_label_policy.json',SETTINGS)
    dump(root/'teacher_provenance.json',{'teacher_manifest_sha256':teacher_hash,
        'checkpoint_path_at_creation':str(checkpoint.resolve()),'checkpoint_manifest':json.loads((checkpoint/'checkpoint-manifest.json').read_text()),
        'deployment_model':'front-camera connectome (not trained in this build)','runtime':'PyTorch float32 '+device,
        'torch_version':torch.__version__,'reference_image_size_wh':[640,360],
        'training_overlap_warning':'Teacher was fine-tuned on Dangjin data; independence from these recordings is not established. Pseudo-label evaluation is not expert ground truth.'})
    processor=AutoImageProcessor.from_pretrained(checkpoint,local_files_only=True);processor.size={'height':360,'width':640}
    model,info=MaskFormerForInstanceSegmentation.from_pretrained(checkpoint,local_files_only=True,use_safetensors=True,output_loading_info=True)
    if any(info.get(k) for k in ['missing_keys','unexpected_keys','mismatched_keys','error_msgs']):raise ValueError('Teacher did not load strictly')
    model=model.to(device).eval().requires_grad_(False)
    cal=json.loads((root/'calibration_assumptions.json').read_text())
    items=sorted([i for i in json.loads((root/'source_inventory.json').read_text()) if 'duplicate_of' not in i],key=lambda i:int(i['start_ns']))
    statuses=[]
    for item in items:
        _,sid=source_ids(item);directory=root/'sequences'/sid
        while not (directory/'completed.json').exists():
            if not watch:raise FileNotFoundError('Source extraction incomplete: '+sid)
            time.sleep(5)
        statuses.append(process_sequence(root,directory,model,processor,device,cal,teacher_hash))
    with (root/'pseudo_labels.jsonl.tmp').open('w') as f:
        for status in statuses:
            with (root/'sequences'/status['sequence']/'pseudo_labels.jsonl').open() as part:
                for record in part:f.write(record)
    (root/'pseudo_labels.jsonl.tmp').replace(root/'pseudo_labels.jsonl')
    dump(root/'labeling_complete.json',{'status':'complete','sequences':statuses,'samples':sum(s['samples'] for s in statuses)})


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--dataset',type=Path,required=True)
    p.add_argument('--teacher',type=Path,required=True);p.add_argument('--device',choices=['cpu','mps','cuda'],default='mps');p.add_argument('--watch',action='store_true')
    a=p.parse_args();run(a.dataset,a.teacher,a.device,a.watch)

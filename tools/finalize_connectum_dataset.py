#!/usr/bin/env python3
"""Validate extracted samples and build session splits, temporal clips and index."""
import argparse
from collections import Counter,defaultdict
import hashlib
import io
import json
from pathlib import Path
import time

import numpy as np
from PIL import Image

from build_connectum_dataset import dump,line,source_ids

SPLITS={'20260915_153503':'train','20261021_163415':'train','20261021_171114':'val',
        '20261021_170306':'test','20261021_163130':'review'}


def temporal_clips(rows,labels,length=8,max_gap=.4):
    window=[];clips=[];previous=None
    for row in rows:
        stamp=int(row['camera']['header_time_ns'])
        if previous is not None and (row['sequence']!=previous['sequence'] or not 0<(stamp-int(previous['camera']['header_time_ns']))/1e9<=max_gap):
            window=[]
        window.append(row);window=window[-length:];previous=row
        if len(window)==length and row['id'] in labels:
            clips.append({'id':row['id'],'session':row['session'],'sequence':row['sequence'],
                'split':SPLITS[row['session']],'frame_ids':[r['id'] for r in window],
                'dt_s':[0.]+[(int(b['camera']['header_time_ns'])-int(a['camera']['header_time_ns']))/1e9 for a,b in zip(window,window[1:])],
                'label_frame_id':row['id'],'is_pseudo':True,'human_verified':False,
                'command_available':labels[row['id']]['estimated_command'] is not None,
                'state_reset_at_clip_start':True})
    return clips


def audit_sequence(root,directory):
    report_path=directory/'validation.json'
    completed=json.loads((directory/'completed.json').read_text());labeled=json.loads((directory/'labels_completed.json').read_text())
    if report_path.exists():
        report=json.loads(report_path.read_text())
        if report['raw_index_sha256']==completed['index_sha256'] and report['label_index_sha256']==labeled['index_sha256'] and report['passed']:
            return report
    rows=[json.loads(s) for s in (directory/'samples.jsonl').read_text().splitlines()]
    labels=[json.loads(s) for s in (directory/'pseudo_labels.jsonl').read_text().splitlines()]
    by_id={r['id']:r for r in rows};hashes={};previous=0;errors=[];gaps=[];clouds=set()
    for row in rows:
        try:
            assert row['recorded_command'] is None and row['observed_motion'] is None
            assert row['camera_views']==['front'] and not row['is_360_camera']
            t=int(row['camera']['header_time_ns']);assert t>previous
            if previous and t-previous>400_000_000:gaps.append({'before_id':row['id'],'gap_s':(t-previous)/1e9})
            previous=t;camera=row['camera'];path=camera['image'];data=(root/path).read_bytes()
            digest=hashlib.sha256(data).hexdigest();assert digest==camera['sha256'];hashes[path]=digest
            with Image.open(io.BytesIO(data)) as image:
                assert image.size==(camera['width'],camera['height']);image.verify()
            if camera['index'] in (0,len(rows)-1):
                with Image.open(io.BytesIO(data)) as image:assert hashlib.sha256(image.convert('RGB').tobytes()).hexdigest()==camera['decoded_rgb_sha256']
            cloud=row['lidar']
            if cloud:
                delta=(int(cloud['midpoint_ns'])-t)/1e6
                assert abs(delta-cloud['camera_delta_ms'])<1e-6
                assert cloud['association_valid']==(abs(delta)<=60)
                path=cloud['path']
                if path not in clouds:
                    data=(root/path).read_bytes();digest=hashlib.sha256(data).hexdigest();assert digest==cloud['sha256'];hashes[path]=digest
                    with np.load(io.BytesIO(data)) as a:
                        assert a['xyz_sensor_m'].shape==(cloud['point_count'],3)
                        assert a['point_timestamp_ns'].shape==a['valid_point_mask'].shape==(cloud['point_count'],)
                        assert int(a['valid_point_mask'].sum())==cloud['valid_point_count']
                        valid=np.isfinite(a['xyz_sensor_m']).all(1)&(np.linalg.norm(a['xyz_sensor_m'],axis=1)>1e-6)
                        assert np.array_equal(valid,a['valid_point_mask'])
                        assert int(a['point_timestamp_ns'].min())==int(cloud['scan_start_ns'])
                        assert int(a['point_timestamp_ns'].max())==int(cloud['scan_end_ns'])
                        for name in cloud['additional_fields']:assert len(a[name])==cloud['point_count']
                    clouds.add(path)
        except Exception as error:errors.append({'id':row['id'],'error':str(error) or type(error).__name__})
    shards=defaultdict(list)
    for label in labels:shards[label['labels']['road']['archive']].append(label)
    for path,group in shards.items():
        try:
            payload=(root/path).read_bytes();hashes[path]=hashlib.sha256(payload).hexdigest()
            with np.load(io.BytesIO(payload)) as shard:
                arrays={k:shard[k] for k in ['semantic','road','confidence','lidar_ground']}
            for label in group:
                assert label['id'] in by_id
                row=by_id[label['id']]
                assert label['recorded_command'] is None and label['is_estimated'] and not label['human_verified']
                for kind,ref in label['labels'].items():
                    value=arrays[ref['array']][ref['index']]
                    assert value.shape==(360,640) and value.dtype==np.uint8
                    assert hashlib.sha256(value.tobytes()).hexdigest()==label['label_sha256'][kind]
                    if kind in ['road','lidar_ground']:assert set(np.unique(value))<={0,1,255}
                    if kind=='semantic':assert value.max()<65
                    if kind=='lidar_ground' and row['lidar'] is None:assert (value==255).all()
                command=label['estimated_command']
                if command:
                    assert row['lidar'] is not None and row['lidar']['association_valid']
                    assert command['is_estimated'] and not command['human_verified'] and not command['deployable']
                    assert 0<=command['linear_x_mps']<=.25 and abs(command['angular_z_radps'])<=.18
                    assert command['linear_y_mps']==0
                    if command['stop']:assert command['linear_x_mps']==command['angular_z_radps']==0
                    if command['action']=='turn_right':assert command['angular_z_radps']<0
                    if command['action']=='turn_left':assert command['angular_z_radps']>0
        except Exception as error:errors.append({'shard':path,'error':str(error) or type(error).__name__})
    expected_images={r['camera']['image'] for r in rows}
    actual_images={p.relative_to(root).as_posix() for p in (directory/'images').glob('*.png') if not p.name.startswith('.')}
    assert expected_images==actual_images,'Unexpected image files in '+str(directory)
    quarantine_path=directory/'quarantined_scans.json'
    quarantined=json.loads(quarantine_path.read_text()) if quarantine_path.exists() else []
    for entry in quarantined:
        try:
            path=entry['path'];payload=(root/path).read_bytes();digest=hashlib.sha256(payload).hexdigest()
            assert digest==entry['sha256'];hashes[path]=digest
            assert not entry['used_for_association'] and path not in clouds
            with np.load(io.BytesIO(payload)) as cloud:
                assert cloud['xyz_sensor_m'].shape==(entry['point_count'],3)
                assert len(cloud['point_timestamp_ns'])==entry['point_count']
                for field in entry['additional_fields']:assert len(cloud[field])==entry['point_count']
        except Exception as error:errors.append({'quarantine':entry['path'],'error':str(error) or type(error).__name__})
    report={'passed':not errors,'sequence':directory.name,'samples':len(rows),'labels':len(labels),'errors':errors,'gaps_over_400ms':gaps,
        'raw_index_sha256':completed['index_sha256'],'label_index_sha256':labeled['index_sha256'],
        'checked_assets':len(hashes),'quarantined_lidar_scans_checked':len(quarantined),'native_rgb_roundtrips_checked':min(2,len(rows)),
        'scope':'All referenced image/cloud/shard hashes, PNG structure, LiDAR fields/time/masks and label values; first/last image exact decoded RGB hash.'}
    dump(report_path,report)
    (directory/'asset_checksums.txt').write_text(''.join(f'{h}  {p}\n' for p,h in sorted(hashes.items())))
    print(line({'event':'validated','sequence':directory.name,'samples':len(rows),'labels':len(labels),'errors':len(errors)}),end='',flush=True)
    return report


def finalize(root,reports):
    if not all(r['passed'] for r in reports):raise ValueError('Validation errors; see per-sequence validation.json')
    rows=[json.loads(s) for s in (root/'samples.jsonl').read_text().splitlines()]
    label_rows=[json.loads(s) for s in (root/'pseudo_labels.jsonl').read_text().splitlines()];labels={r['id']:r for r in label_rows}
    assert len({r['id'] for r in rows})==len(rows)
    clips=temporal_clips(rows,labels)
    split_dir=root/'splits';split_dir.mkdir(exist_ok=True)
    splits={name:[] for name in ['train','val','test','review']}
    for row in rows:
        split=SPLITS[row['session']];splits[split].append(row['id'])
    for name,ids in splits.items():dump(split_dir/f'{name}.json',ids)
    for i,a in enumerate(splits):
        for b in list(splits)[i+1:]:assert not set(splits[a])&set(splits[b])
    dump(split_dir/'policy.json',{'unit':'entire recording session','assignments':SPLITS,'random_frame_split':False,
        'claims':'Session-separated candidate splits; same farm and pseudo labels. Not an expert-annotated independent benchmark.',
        'teacher_training_overlap':'Not fully audited; see teacher_provenance.json',
        'action_validation':'20261021_171114 is held out with LiDAR-backed candidate actions; training actions are from 20260915_153503.',
        'test_scope':'20261021_170306 has no LiDAR-backed action targets. Test is for surface recognition only, not control accuracy.',
        'review_session':'Very short camera startup recording, excluded from training/evaluation examples.'})
    with (root/'clips.jsonl').open('w') as f:
        for clip in clips:f.write(line(clip))
    inventory=json.loads((root/'source_inventory.json').read_text());config=json.loads((root/'extraction_config.json').read_text())
    session_summary=[];viewer=[]
    for session in SPLITS:
        srows=[r for r in rows if r['session']==session];slabs=[l for l in label_rows if l['session']==session]
        session_summary.append({'session':session,'split':SPLITS[session],'samples':len(srows),'pseudo_labeled_frames':len(slabs),
            'with_lidar':sum(r['lidar'] is not None for r in srows),'clips':sum(c['session']==session for c in clips),
            'estimated_actions':dict(Counter(l['estimated_command']['action'] for l in slabs if l['estimated_command']))})
    for report in reports:
        directory=root/'sequences'/report['sequence'];status=json.loads((directory/'completed.json').read_text());ls=json.loads((directory/'labels_completed.json').read_text())
        viewer.append({'sequence':report['sequence'],'session':status['session'],'split':SPLITS[status['session']],
            'samples':status['samples'],'labels':ls['samples'],'has_lidar':status['has_lidar'],'commands':ls['commands'],
            'previews':json.loads((directory/'previews'/'index.json').read_text()),
            'raw_index':(directory/'samples.jsonl').relative_to(root).as_posix(),'label_index':(directory/'pseudo_labels.jsonl').relative_to(root).as_posix()})
    report={'status':'complete','schema':'connectum.front.dataset.v1','source_root':'/Volumes/MY_HDD/20261021_dangjin',
        'source_files_found':len(inventory),'duplicate_files_skipped':sum('duplicate_of' in i for i in inventory),
        'unique_source_files':len(reports),'source_bytes_including_duplicates':sum(i['bytes'] for i in inventory),
        'unique_source_duration_s':sum(i['duration_s'] for i in inventory if 'duplicate_of' not in i),
        'samples':len(rows),'pseudo_labeled_frames':len(labels),'context_only_frames':len(rows)-len(labels),'temporal_clips':len(clips),
        'camera_hz_target':config['target_hz'],'label_hz_target':1,'front_camera_only':True,'lossless_images':True,
        'with_lidar':sum(r['lidar'] is not None for r in rows),'camera_only':sum(r['lidar'] is None for r in rows),
        'invalid_lidar_associations':sum(r['lidar'] is not None and not r['lidar']['association_valid'] for r in rows),
        'quarantined_lidar_scans':sum(r.get('quarantined_lidar_scans_checked',0) for r in reports),
        'recorded_commands':0,'expert_verified_labels':0,
        'estimated_actions':dict(Counter(l['estimated_command']['action'] for l in label_rows if l['estimated_command'])),
        'command_abstentions':dict(Counter(l['command_status'] for l in label_rows if l['estimated_command'] is None)),
        'sessions':session_summary,'validation_passed':True,'human_review_pending':True,'connectome_training_run':False,
        'limits':['Pseudo-labels are not ideal/expert action ground truth.', 'Unverified LiDAR extrinsics, no deskew.',
                  'Surface masks do not encode farm access restrictions.', 'Teacher training overlap has not been fully audited.',
                  'No synthetic missing detections injected; actual time gaps retained.',
                  'Mixed-time LiDAR scans over 200 ms are preserved in quarantine and excluded from association.']}
    dump(root/'dataset.json',report)
    dump(root/'validation_report.json',{'passed':True,'sequences':len(reports),'samples':len(rows),'pseudo_labels':len(labels),
        'checked_assets':sum(r['checked_assets'] for r in reports),'native_rgb_roundtrips_checked':sum(r['native_rgb_roundtrips_checked'] for r in reports),
        'split_leakage_check':'no sample/session overlap between assigned splits; does not assert teacher-data independence',
        'not_validated':['expert label accuracy','recording-specific LiDAR extrinsics','closed-loop real-robot driving']})
    (root/'viewer-data.js').write_text('window.CONNECTUM_DATA = '+json.dumps({'summary':report,'sequences':viewer},ensure_ascii=False)+';\n')
    print(line({'event':'dataset_complete',**report}),end='',flush=True)


def run(root,watch):
    items=sorted([i for i in json.loads((root/'source_inventory.json').read_text()) if 'duplicate_of' not in i],key=lambda i:int(i['start_ns']))
    reports=[]
    for item in items:
        _,sid=source_ids(item);directory=root/'sequences'/sid
        while not (directory/'labels_completed.json').exists():
            if not watch:raise FileNotFoundError('Labels not ready: '+sid)
            time.sleep(5)
        reports.append(audit_sequence(root,directory))
    while not (root/'extraction_complete.json').exists() or not (root/'labeling_complete.json').exists():
        if not watch:raise FileNotFoundError('Global indexes not ready')
        time.sleep(5)
    finalize(root,reports)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--dataset',type=Path,required=True);p.add_argument('--watch',action='store_true')
    a=p.parse_args();run(a.dataset,a.watch)

#!/usr/bin/env python3
"""Check sample artifact consistency, provenance separation and sensor matching."""
import argparse
import hashlib
import json
from collections import Counter
from pathlib import Path

import numpy as np
from PIL import Image


def validate(root):
    rows=[json.loads(s) for s in (root/'labels.jsonl').read_text().splitlines()]
    raw=[json.loads(s) for s in (root/'raw_samples.jsonl').read_text().splitlines()]
    source=json.loads((root/'sources.json').read_text())
    assert len(rows)==len(raw)>0
    assert len({r['id'] for r in rows})==len(rows)
    assert all(not s['command_topics_present'] for s in source)
    allowed=[s['file'] for s in source]
    previous={};errors=[]
    for r,original in zip(rows,raw):
        try:
            assert all(r[k]==v for k,v in original.items()), 'raw metadata modified'
            assert r['source_file'] in allowed
            assert r['recorded_command'] is None and r['observed_motion'] is None
            assert r['estimated_command']['is_estimated'] and not r['estimated_command']['human_verified']
            assert not r['training']['approved_for_training'] and r['training']['split']=='review_only'
            assert r['training']['camera_views']==['front']
            assert r['training']['group_id']==r['source_file']
            assert not r['geometry']['extrinsic_verified']
            if r['gps_velocity'] is not None:
                assert not r['gps_velocity']['usable_for_temporal_training']
                assert abs(r['gps_velocity']['header_minus_log_s'])>1000
            c=r['estimated_command'];assert 0<=c['linear_x_mps']<=.25
            assert abs(c['angular_z_radps'])<=.18 and c['linear_y_mps']==0
            assert c['stop']==(c['action']=='stop')
            if c['stop']:assert c['linear_x_mps']==0 and c['angular_z_radps']==0
            if c['action']=='turn_right':assert c['angular_z_radps']<0
            if c['action']=='turn_left':assert c['angular_z_radps']>0
            if c['stop_reason']=='insufficient_ground_support':
                assert 'insufficient_lidar_ground_support' in r['training']['exclusion_reasons']
            camera_time=int(r['camera']['header_time_ns'])
            assert camera_time>previous.get(r['episode'],0)
            previous[r['episode']]=camera_time
            with Image.open(root/r['camera']['image']) as im:
                assert im.size==(1280,720);im.verify()
            for key in ['review_camera','review_bev']:
                with Image.open(root/r[key]) as im:im.verify()
            a=np.load(root/r['lidar']['path'])
            assert len(a['xyz_sensor_m'])==len(a['point_timestamp_ns'])==len(a['valid_point_mask'])==r['lidar']['point_count']
            assert int(a['valid_point_mask'].sum())==r['lidar']['valid_point_count']
            np.testing.assert_array_equal(a['valid_point_mask'],np.isfinite(a['xyz_sensor_m']).all(1)&(np.linalg.norm(a['xyz_sensor_m'],axis=1)>1e-6))
            assert int(a['point_timestamp_ns'].min())==int(r['lidar']['scan_start_ns'])
            assert int(a['point_timestamp_ns'].max())==int(r['lidar']['scan_end_ns'])
            delta=(int(r['lidar']['acquisition_midpoint_ns'])-camera_time)/1e6
            assert abs(delta-r['lidar']['camera_delta_ms'])<1e-6
            assert abs(delta)<=60 and r['lidar']['association_valid']
            assert len(r['camera']['message_sha256'])==len(r['lidar']['message_sha256'])==64
            assert r['geometry']['ground_plane']['valid']
        except (AssertionError,ValueError,OSError) as exc:
            errors.append({'id':r['id'],'error':str(exc)})
    report={'passed':not errors,'samples_checked':len(rows),'errors':errors,
            'actions':dict(Counter(r['estimated_command']['action'] for r in rows)),
            'checks':['raw evidence unchanged','PNG readability and dimensions','LiDAR point/time/mask alignment',
                      'camera/LiDAR matching arithmetic','chronological per-episode timestamps',
                      'estimated vs absent recorded command separation','GPS clock rejection',
                      'command units/bounds/sign/stop consistency','review-only training status'],
            'not_validated':['expert correctness of labels','recording-specific extrinsic calibration',
                             'closed-loop driving success','360-degree recognition','collision-free operation']}
    (root/'validation_report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    return report


def manifest(root):
    paths=sorted(p for p in root.rglob('*') if p.is_file() and p.name not in ['SHA256SUMS','.DS_Store'])
    lines=[]
    for path in paths:
        h=hashlib.sha256()
        with path.open('rb') as f:
            for block in iter(lambda:f.read(1024*1024),b''):h.update(block)
        lines.append(f'{h.hexdigest()}  {path.relative_to(root).as_posix()}')
    (root/'SHA256SUMS').write_text('\n'.join(lines)+'\n')
    return len(lines)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--dataset',type=Path,required=True)
    p.add_argument('--write-manifest',action='store_true');a=p.parse_args()
    r=validate(a.dataset)
    if a.write_manifest:r['hashed_files']=manifest(a.dataset)
    print(json.dumps(r,ensure_ascii=False,indent=2))
    raise SystemExit(0 if r['passed'] else 1)

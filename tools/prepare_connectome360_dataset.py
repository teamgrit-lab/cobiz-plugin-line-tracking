"""Prepare real 360 RGB, partial road labels and explicitly unapproved action proposals."""
import argparse
from collections import Counter, defaultdict
from concurrent.futures import ThreadPoolExecutor
from fractions import Fraction
import hashlib
import json
import re
import shutil
import subprocess
from pathlib import Path

import cv2
import numpy as np

from camera_trajectory import track_camera_path
from fcn360_road import FcnRoadPlanner, ROAD
from local_path import LocalPathConfig
from surround_camera import load_profile, profile_sha256, project_panorama, view_atlas

ORDER = ['front', 'left', 'rear', 'right']


def sha(path):
    with path.open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()


def write_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n')


def jsonl(path):
    return [json.loads(line) for line in path.read_text().splitlines()]


def partition(clip, stamp):
    if clip == 'ID482':
        return 'train' if stamp < 1170 else 'development' if stamp >= 1210 else 'gap'
    return 'test' if clip == 'ID403' else 'validation'


def identify(name):
    for clip in ('ID403', 'ID482'):
        if clip in name:
            return clip
    match = re.search(r'camera \((\d)\)', name)
    if not match:
        raise ValueError(f'Unknown recording: {name}')
    return 'RSSP' + match[1].zfill(2)


def decode(source, out):
    clip = source['recording']
    folder = out / 'panoramas' / clip
    folder.mkdir(parents=True, exist_ok=True)
    limit = 1500. if clip == 'ID482' else float('inf')
    sample = 'isnan(prev_selected_t)+gt(floor(5*t+0.00000001),floor(5*prev_selected_t+0.00000001))'
    expression = f'lt(t,1500)*({sample})' if clip == 'ID482' else sample
    cmd = ['ffmpeg', '-y', '-nostdin', '-hide_banner', '-nostats', '-v', 'info',
           '-hwaccel', 'videotoolbox', '-i', source['path'], *(['-t', '1500'] if clip=='ID482' else []),
           '-map', '0:v:0', '-an', '-vf', f"select='{expression}',showinfo=checksum=0",
           '-fps_mode', 'passthrough', '-q:v', '2', '-threads', '1', str(folder / '%06d.jpg')]
    log = out / 'provenance' / (clip + '-decode.log')
    previous=log.read_text(errors='replace') if log.exists() else ''
    ticks=[int(x) for x in re.findall(r'\bn:\s*\d+\s+pts:\s*(-?\d+)\s+pts_time:', previous)]
    time_base = Fraction(source['time_base'])
    expected=[p for p in ticks if float(p*time_base)<limit]
    complete='Lsize=' in previous and len(expected)==len(list(folder.glob('[0-9]*.jpg')))
    if not complete:
        with log.open('w') as handle:
            process = subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=handle)
        if process.returncode:
            raise ValueError(f'Decode failed: {clip}; see {log}')
    text = log.read_text(errors='replace')
    pts = [int(x) for x in re.findall(r'\bn:\s*\d+\s+pts:\s*(-?\d+)\s+pts_time:', text)]
    pts = [p for p in pts if float(p*time_base)<limit]
    frames = sorted(folder.glob('[0-9]*.jpg'))
    if len(pts) != len(frames):
        raise ValueError(f'Decode failed: {clip}; see {log}')
    time_base = Fraction(source['time_base'])
    rows = []
    for frame, tick in zip(frames, pts):
        stamp = float(tick * time_base)
        if stamp >= limit:
            frame.unlink()
            continue
        rows.append(dict(id=f'{clip}_{tick:010d}', recording=clip, source_sha256=source['sha256'],
                         source_pts=tick, source_time_sec=stamp, time_base=source['time_base'],
                         time_bin=int(tick * time_base * 5), split=partition(clip, stamp),
                         panorama=str(frame.relative_to(out)), view_order=ORDER,
                         camera_profile_sha256=source['camera_profile_sha256'],
                         executed_command=None, robot_pose=None, lidar=None))
    if not rows or any(b['source_pts'] <= a['source_pts'] for a, b in zip(rows, rows[1:])):
        raise ValueError(f'Invalid timestamps for {clip}')
    if len({r['time_bin'] for r in rows}) != len(rows):
        raise ValueError('Repeated source bin')
    print('Decoded', clip, len(rows), 'actual frames', flush=True)
    return rows


def project_row(row, out, profile):
    image = cv2.imread(str(out / row['panorama']))
    atlas = view_atlas(project_panorama(image, profile))
    dest = out / 'atlases' / row['recording'] / (row['id'] + '.png')
    if not cv2.imwrite(str(dest), atlas, [cv2.IMWRITE_PNG_COMPRESSION, 1]):
        raise OSError(dest)
    row['views_atlas'] = str(dest.relative_to(out))
    row['panorama_sha256'], row['atlas_sha256'] = sha(out/row['panorama']), sha(dest)
    return row


def import_labels(root, out, sources, profile):
    entries = jsonl(root / 'manifest.jsonl')
    source_index = json.loads((root / 'sources.json').read_text())['sources']
    mapping = {s['recording']: s for s in sources if s['used']}
    targets = out/'road/annotations.jsonl'
    counts = Counter()
    for d in ('road/panoramas', 'road/atlases', 'road/semantic', 'road/binary'):
        (out/d).mkdir(parents=True, exist_ok=True)
    with targets.open('w') as handle:
        grouped = defaultdict(list)
        for entry in entries:
            if entry['semantic_label']:
                grouped[entry['group_id']].append(entry)
        for gid, group in grouped.items():
            first = group[0]
            clip = 'ID482' if first['clip']=='ID482' else first['clip'].upper().replace('CLIP','RSSP')
            if source_index[first['source_key']]['sha256'] != mapping[clip]['sha256']:
                raise ValueError('Annotated recording hash mismatch')
            tick = first['pts']
            stamp = tick * Fraction(mapping[clip]['time_base'])
            if abs(float(stamp) - first['pts_time_sec']) > .00002:
                raise ValueError('Annotation timestamp mismatch')
            if clip == 'ID482' and not 0 <= stamp < 1500:
                raise ValueError('Annotation exceeds cutoff')
            panorama = root / first['panorama']
            copy = out/'road/panoramas'/panorama.name
            shutil.copyfile(panorama, copy)
            # These masks were drawn on views of this saved JPEG. Keep that JPEG as the supervision reference.
            frames = project_panorama(cv2.imread(str(copy)), profile)
            atlas = out/'road/atlases'/(gid+'.png')
            cv2.imwrite(str(atlas), view_atlas(frames), [cv2.IMWRITE_PNG_COMPRESSION,1])
            for row in group:
                sem = cv2.imread(str(root/row['semantic_label']),0)
                sem = cv2.resize(sem,(480,270),interpolation=cv2.INTER_NEAREST_EXACT)
                binary = np.full((270,480),255,np.uint8)
                known=(sem!=255)&(sem!=0)
                binary[known]=(sem[known]==3)
                conflicts = 0
                if row['binary_road_label']:
                    b=cv2.imread(str(root/row['binary_road_label']),0)
                    b=cv2.resize(b,(480,270),interpolation=cv2.INTER_NEAREST_EXACT)
                    valid=b!=255
                    conflict=known&valid&(binary!=b)
                    binary[valid]=b[valid]
                    binary[conflict]=255
                    conflicts=int(conflict.sum())
                for folder,mask in [('semantic',sem),('binary',binary)]:
                    cv2.imwrite(str(out/'road'/folder/(row['id']+'.png')),mask)
                part=partition(clip,float(stamp))
                result=dict(id=row['id'],recording=clip,source_pts=tick,source_time_sec=float(stamp),
                            split=part,view=row['view'],view_index=ORDER.index(row['view']),
                            panorama=str(copy.relative_to(out)),views_atlas=str(atlas.relative_to(out)),
                            semantic_label=f"road/semantic/{row['id']}.png",road_label=f"road/binary/{row['id']}.png",
                            label_origin='inherited_visual_partial',human_verified=False,
                            source_annotation_id=row['id'],label_conflict_pixels_ignored=conflicts,
                            valid_pixels=int((binary!=255).sum()),road_pixels=int((binary==1).sum()),
                            vegetation_pixels=int((sem==11).sum()))
                counts[part]+=1
                handle.write(json.dumps(result,ensure_ascii=False)+'\n')
    for name in ['schema.json','annotations/semantic-polygons.json','annotations/binary-polygons.json']:
        shutil.copyfile(root/name,out/'provenance'/Path(name).name)
    return dict(counts)


def prepare_proposals(rows, previous, out, sources):
    info=json.loads((previous/'sources.json').read_text())
    index={s['recording']:s for s in sources if s['used']}
    for clip, source in index.items():
        if not any(s['sha256']==source['sha256'] and s['canonical']==clip for s in info.values()):
            raise ValueError('Proposal recording hash mismatch')
    latest=FcnRoadPlanner(LocalPathConfig(),1.6)
    stats=Counter()
    report={}
    for clip in index:
        records=jsonl(previous/'records'/(clip+'.jsonl'))
        with np.load(previous/'records'/(clip+'-classes.npz')) as stored:
            print('Proposal cache',clip,stored.files,flush=True)
            masks=stored['class_ids']
            if masks.shape != (len(stored['pts']),4,8,16):
                raise ValueError('Unexpected proposal grid shape')
            lookup={int(tick):i for i,tick in enumerate(stored['pts'])}
            if set(lookup) != {r['source_pts'] for r in records if r['fresh']}:
                raise ValueError('Proposal frame index differs from its audit log')
            selected=[]; ids=[]; prev=None
            with (out/'policy'/f'{clip}-proposals.jsonl').open('w') as f:
                for row in (r for r in rows if r['recording']==clip):
                    ix=lookup.get(row['source_pts'])
                    if ix is None:
                        stats['unmatched']+=1
                        continue
                    label=masks[ix].copy()
                    if prev is None or row['time_bin'] != prev['time_bin']+1 or row['split']!=prev['split']:
                        latest.reset()
                    estimate,metrics=latest.plan(label,stamp=row['source_time_sec'])
                    cmd=track_camera_path(estimate.points_xy if estimate else None,0.,.8)
                    proposal=dict(id=row['id'],source_pts=row['source_pts'],source_time_sec=row['source_time_sec'],
                                  split=row['split'],executed_command=None,expert_command=None,
                                  estimated_command=dict(vx_mps=cmd.vx,vy_mps=cmd.vy,yaw_radps=cmd.yaw_rate,
                                                         stop=cmd.vx==0.,reason=cmd.reason),
                                  estimate_origin='FCN pseudo mask -> offline Road planner -> camera-path teacher',
                                  human_verified=False,approved=False,action_valid=False,
                                  geometry_estimated=dict(camera_height_m=1.6,pitch_deg=0.),
                                  normalization=dict(vx_mps=.8,yaw_radps=.35),front_path_valid=estimate is not None,
                                  proposal_mask_archive=f'policy/{clip}-pseudo-masks.npz',proposal_mask_index=len(selected))
                    f.write(json.dumps(proposal,ensure_ascii=False)+'\n')
                    selected.append(label);ids.append(row['id']);prev=row
                    stats['candidate_commands']+=1
                    stats['candidate_stop' if cmd.vx==0 else 'candidate_move']+=1
            np.savez_compressed(out/'policy'/f'{clip}-pseudo-masks.npz',
                                ids=np.asarray(ids),classes=np.asarray(selected,np.uint8))
            report[clip]=len(ids)
    with (out/'road/pseudo-labels.jsonl').open('w') as handle:
        for file in sorted((out/'policy').glob('[A-Z]*-proposals.jsonl')):
            for proposal in jsonl(file):
                record = dict(id=proposal['id'], split=proposal['split'],
                              class_archive=proposal['proposal_mask_archive'],
                              array_index=proposal['proposal_mask_index'],
                              shape=[4,8,16], road_class=3, model_confidence=None,
                              origin='cached_FCN_teacher_prediction', is_pseudo=True,
                              human_verified=False, approved=False,
                              detector_sha256='6c0c39112cf2f39b65904c27f313cad4a07c2369ef464e953cb3df1e9ccf0671')
                handle.write(json.dumps(record)+'\n')
    return dict(stats),report


def sequences(rows,out):
    by=defaultdict(list)
    for row in rows:
        by[row['recording']].append(row)
    total=0
    with (out/'policy/sequences.jsonl').open('w') as handle:
        for clip, group in by.items():
            for i in range(9,len(group)):
                segment=group[i-9:i+1]
                if len({r['split'] for r in segment})!=1 or segment[0]['split']=='gap':
                    continue
                if any(b['time_bin']!=a['time_bin']+1 or not 0 < b['source_time_sec']-a['source_time_sec'] <= .45
                       for a,b in zip(segment,segment[1:])):
                    continue
                record=dict(id=segment[-1]['id'],recording=clip,split=segment[0]['split'],
                            frame_ids=[r['id'] for r in segment],
                            dt_s=[b['source_time_sec']-a['source_time_sec'] for a,b in zip(segment,segment[1:])],
                            target_frame=segment[-1]['id'],state_reset=True,executed_command=None,
                            action_target=None,action_valid=False)
                handle.write(json.dumps(record)+'\n')
                total+=1
    return total


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source',type=Path,required=True)
    p.add_argument('--labels',type=Path,required=True)
    p.add_argument('--proposals',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--resume',action='store_true',help='Reuse completed RGB extraction after a failed metadata step')
    args=p.parse_args()
    out=args.output
    out.mkdir(parents=True,exist_ok=args.resume)
    for name in ('provenance','panoramas','atlases','policy','road','splits','preview'):
        (out/name).mkdir(exist_ok=args.resume)
    cv2.setNumThreads(1)
    profile=load_profile()
    write_json(out/'camera-profile.json',profile)
    sources=[];seen={}
    for path in sorted(args.source.glob('*.mp4')):
        if path.name.startswith('.'):
            continue
        metadata=json.loads(subprocess.check_output(['ffprobe','-v','error','-select_streams','v:0',
            '-show_entries','stream=codec_name,width,height,time_base,duration,nb_frames','-of','json',str(path)]))['streams'][0]
        if (metadata['width'],metadata['height'])!=(2880,1440):
            raise ValueError('Unexpected source projection dimensions')
        key=sha(path)
        alias=seen.get(key)
        s=dict(path=str(path.resolve()),sha256=key,recording=identify(path.name),
               size_bytes=path.stat().st_size,duration_sec=float(metadata['duration']),**metadata,
               used=alias is None,duplicate_of=alias,camera_profile_sha256=profile_sha256(profile))
        if alias is None:
            seen[key]=path.name
        else:
            s['recording']=identify(alias)
        sources.append(s)
    write_json(out/'sources.json',sources)
    if len({s['recording'] for s in sources if s['used']}) != sum(s['used'] for s in sources):
        raise ValueError('Multiple different encodings in a recording; choose canonical sources explicitly')
    used=[s for s in sources if s['used']]
    complete=out/'rgb-complete.json'
    if args.resume and (out/'frames.jsonl').is_file():
        rows=jsonl(out/'frames.jsonl')
        for source in used:
            clip=source['recording']
            exported=list((out/'panoramas'/clip).glob('[0-9]*.jpg'))
            if len(exported)!=sum(r['recording']==clip for r in rows):
                raise ValueError('RGB export is incomplete')
        for row in rows:
            if not (out/row['views_atlas']).is_file():
                raise ValueError('Missing RGB atlas in resumed export')
        print('Reusing completed RGB export',len(rows),flush=True)
    else:
        with ThreadPoolExecutor(max_workers=2) as workers:
            decoded=list(workers.map(lambda s:decode(s,out),used))
        rows=[r for group in decoded for r in group]
        for source in used:
            (out/'atlases'/source['recording']).mkdir()
        with ThreadPoolExecutor(max_workers=4) as workers, (out/'frames.jsonl').open('w') as handle:
            for i,row in enumerate(workers.map(lambda r:project_row(r,out,profile),rows)):
                handle.write(json.dumps(row,ensure_ascii=False)+'\n')
                if i%500==0:
                    print('Projected',i+1,'/',len(rows),'four-view RGB atlases',flush=True)
    write_json(complete,dict(frames=len(rows),sources_sha256=sha(out/'sources.json'),manifest_sha256=sha(out/'frames.jsonl')))
    labels=import_labels(args.labels,out,sources,profile)
    nseq=sequences(rows,out)
    for part in ('train','development','validation','test','gap'):
        (out/'splits'/f'{part}.txt').write_text(''.join(r['id']+'\n' for r in rows if r['split']==part))
    proposed,by_clip=prepare_proposals(rows,args.proposals,out,sources)
    write_json(out/'policy/fault-augmentation-plan.json',
               dict(whole_mask_loss_probability=.16,partial_mask_holes_probability=.64,
                    train_seed_range=[1000,1999],validation_seed_range=[2000,2999],
                    reserved_evaluation_seed=9027,whole_mask_loss_target='stop',
                    source='synthetic augmentation only; not an observed robot stop'))
    write_json(out/'policy/training-contract.json',
               dict(input='four masks predicted by the selected frozen Connectome detector',
                    view_shape=[4,270,480],input_adapter='connectome_policy.policy_pixels',
                    actor_input_shape=[4,32,64],crop_rows=[135,257],
                    action_normalization=dict(vx_mps=.8,yaw_radps=.35),stop_head=True,
                    require_matching_detector_sha256=True,executed_actions_available=False,
                    estimated_actions_require_explicit_opt_in=True,
                    actor_targets='policy/*-proposals.jsonl; review before using as expert labels',
                    road_targets='road/annotations.jsonl; ignore mask value 255',
                    temporal_windows='policy/sequences.jsonl, 10 frames, no frame duplication or gap bridging'))
    summary=dict(schema='connectome360.real_dataset.v1',source_directory=str(args.source.resolve()),
                 source_files=len(sources),unique_recordings=len(used),frames=len(rows),direction_views=len(rows)*4,
                 frame_splits=dict(Counter(r['split'] for r in rows)),semantic_label_views=labels,
                 sequences=nseq,estimated_policy=proposed,policy_proposals_per_recording=by_clip,
                 executed_action_targets=0,human_verified_actions=0,
                 source_derived_images=True,id482_exclusive_cutoff_sec=1500.,
                 camera_profile_sha256=profile_sha256(profile),nominal_sampling_hz=5,
                 sampling='first actual decoded frame per occupied 0.2 s PTS bin; gaps remain gaps',
                 label_provenance=str(args.labels.resolve()),proposal_provenance=str(args.proposals.resolve()),
                 proposal_model_sha256='6c0c39112cf2f39b65904c27f313cad4a07c2369ef464e953cb3df1e9ccf0671')
    write_json(out/'summary.json',summary)
    shutil.copyfile(__file__,out/'provenance'/Path(__file__).name)
    print(json.dumps(summary,ensure_ascii=False),flush=True)


if __name__=='__main__':
    main()

#!/usr/bin/env python3
"""Conservative road-only training selection. Never deletes source MCAP/images."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import shutil
import cv2
import numpy as np


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--dataset', type=Path, default=Path('/Volumes/MY_HDD/connectum_data'))
    p.add_argument('--output', type=Path, default=Path('/Volumes/MY_HDD/connectum_data_selections/front-road-only-v2'))
    p.add_argument('--cache', type=Path, default=Path('.cache/front-road-v2'))
    p.add_argument('--source-cache', type=Path, default=Path('.cache/front-road-v1'))
    a = p.parse_args(); a.output.mkdir(parents=True, exist_ok=True); a.cache.mkdir(parents=True, exist_ok=True)
    samples = list(map(json.loads, (a.dataset / 'samples.jsonl').read_text().splitlines()))
    by_id = {r['id']: r for r in samples}
    labels = list(map(json.loads, (a.dataset / 'pseudo_labels.jsonl').read_text().splitlines()))
    assignment = json.loads((a.dataset / 'splits/policy.json').read_text())['assignments']
    reasons = {}; scores = []; archive = None; sems = None; seeds = []
    for r in labels:
        sample = by_id[r['id']]; seq = sample['sequence']; index = sample['camera']['index']
        ref = r['labels']['semantic']; name = ref['archive']
        if name != archive:
            with np.load(a.dataset / name) as z: sems = z['semantic']
            archive = name
        sidewalk = np.isin(sems[ref['index']], [11, 15])
        whole = float(sidewalk.mean()); near = float(sidewalk[200:350, 160:480].mean())
        why = []
        # The scene review identified paved pedestrian/road transitions in
        # this end-of-recording group. Exclude whole source segments, not just
        # a few high-confidence teacher detections.
        if sample['session'] == '20260915_153503' and int(seq.rsplit('p', 1)[1]) >= 30:
            why.append('mixed_pedestrian_road_source_segment_conservatively_excluded')
        if seq == '20260915_153503_p000':
            why.append('bare_ground_start_not_road_driving')
        if seq == '20261021_170306_p000' and index <= 409:
            why.append('visually_identified_sidewalk_walk_and_transition')
        if whole > .005 or near > .005:
            why.append('teacher_sidewalk_or_pedestrian_area_over_0.5_percent')
        scores.append({'id': r['id'], 'whole_fraction': whole, 'near_fraction': near, 'reasons': why})
        if why:
            seeds.append((sample['session'], int(sample['camera']['log_time_ns']), r['id']))
            reasons[r['id']] = why
    for sample in samples:
        stamp = int(sample['camera']['log_time_ns'])
        if any(session == sample['session'] and abs(stamp - t) <= 3_000_000_000 for session, t, _ in seeds):
            reasons.setdefault(sample['id'], ['within_3_seconds_of_excluded_frame'])
    old = json.loads((a.source_cache / 'entries.json').read_text())
    previous = json.loads((a.source_cache / 'features-progress.json').read_text())
    assert previous['index_sha256'] == hashlib.sha256(json.dumps(old, sort_keys=True).encode()).hexdigest(), 'stale source feature index'
    assert previous['completed'] == len(old), 'source features incomplete'
    chosen = [i for i, r in enumerate(old) if r['id'] not in reasons]; rows = [old[i] for i in chosen]
    # Circuit activations depend on image pixels and a frozen circuit only;
    # selecting precomputed activations does not reuse the trained v1 heads.
    for name in ['features.npy', 'targets.npy']:
        src = np.load(a.source_cache / name, mmap_mode='r')
        assert len(src) == len(old), 'source feature/label length mismatch'
        dst = np.lib.format.open_memmap(a.cache / name, mode='w+', dtype=src.dtype, shape=(len(chosen), *src.shape[1:]))
        for start in range(0, len(chosen), 64): dst[start:start+64] = src[chosen[start:start+64]]
        dst.flush()
    (a.cache / 'entries.json').write_text(json.dumps(rows, indent=2))
    identity = hashlib.sha256(json.dumps(rows, sort_keys=True).encode()).hexdigest()
    (a.cache / 'features-progress.json').write_text(json.dumps(dict(previous, index_sha256=identity, completed=len(rows), total=len(rows))))
    manifest = {'schema': 'connectum.road_only.selection.v1', 'excluded_ids': sorted(reasons),
        'exclusion_reasons': reasons, 'source_dataset': str(a.dataset),
        'source_dataset_manifest_sha256': hashlib.sha256((a.dataset / 'dataset.json').read_bytes()).hexdigest(),
        'policy': 'exclude mixed pedestrian source segments + sidewalk walk + teacher candidates, with 3 s padding',
        'teacher_classes': {'11': 'Pedestrian Area', '15': 'Sidewalk'}, 'threshold': .005,
        'visual_review': 'session overviews and highest-sidewalk-fraction contact sheets; conservative group exclusion, not exhaustive human annotation',
        'original_images_deleted': False, 'retrain_from_v1': False,
        'before': dict(Counter(r['source']+':'+r['split'] for r in old)),
        'after': dict(Counter(r['source']+':'+r['split'] for r in rows)),
        'excluded_labeled_frames': sum(r['id'] in reasons for r in old),
        'excluded_context_and_labeled_frames': len(reasons)}
    (a.output / 'exclusions.json').write_text(json.dumps(manifest, indent=2) + '\n')
    (a.output / 'semantic-scores.json').write_text(json.dumps(scores, indent=2) + '\n')
    (a.output / 'selected-training-samples.json').write_text(json.dumps(rows, indent=2) + '\n')
    for status in ['retained', 'excluded']:
        candidates = [r for r in old if r['source']=='real_pseudo' and r['split']=='train' and (r['id'] in reasons) == (status=='excluded')]
        tiles=[]
        for i in np.linspace(0,len(candidates)-1,24).astype(int):
            row=candidates[i];im=cv2.resize(cv2.imread(row['image']),(320,180));tile=np.full((205,320,3),25,np.uint8);tile[:180]=im
            cv2.putText(tile,row['id'],(3,197),cv2.FONT_HERSHEY_SIMPLEX,.36,(255,255,255),1);tiles.append(tile)
        cv2.imwrite(str(a.output/(status+'-training-review.jpg')),np.concatenate([np.concatenate(tiles[j:j+4],1) for j in range(0,24,4)],0))
    shutil.copy2(Path(__file__), a.output / Path(__file__).name)
    print(json.dumps({k:v for k,v in manifest.items() if k not in ['excluded_ids','exclusion_reasons']}, indent=2))


if __name__ == '__main__':
    main()

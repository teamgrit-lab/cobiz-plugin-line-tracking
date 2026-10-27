#!/usr/bin/env python3
"""Compare one extracted frame/scan per session directly against its MCAP."""
import argparse
import hashlib
import json
from pathlib import Path
import time

import numpy as np
from PIL import Image
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory

from build_connectum_dataset import dump, field_arrays, source_ids
from build_road_sample_dataset import point_times, raw_xyz
from rosbag_mcap_to_mp4 import decode_video_frame


def check_source(root, item):
    _, sid = source_ids(item)
    with (root / 'sequences' / sid / 'samples.jsonl').open() as f:
        rows = [json.loads(s) for s in f]
    row = rows[min(10, len(rows) - 1)]
    refs = {'camera': row['camera']}
    if row['lidar']:
        refs['lidar'] = row['lidar']
    wanted = {ref['message_sha256']: name for name, ref in refs.items()}
    checked = {}
    with Path(item['path']).open('rb') as stream:
        reader = make_reader(stream, decoder_factories=[DecoderFactory()])
        stamps = [int(ref['log_time_ns']) for ref in refs.values()]
        for entry in reader.iter_decoded_messages(topics=[ref['topic'] for ref in refs.values()],
                start_time=min(stamps), end_time=max(stamps) + 1):
            digest = hashlib.sha256(entry.message.data).hexdigest()
            name = wanted.get(digest)
            if name is None:
                continue
            m = entry.decoded_message
            ref = refs[name]
            assert str(entry.message.log_time) == ref['log_time_ns']
            if name == 'camera':
                frame = decode_video_frame(m, entry.schema.name, entry.message.log_time)
                rgb = np.frombuffer(frame.payload, np.uint8).reshape(frame.height, frame.width, 3)
                if frame.input_format == 'bgr24':
                    rgb = rgb[:, :, ::-1]
                with Image.open(root / ref['image']) as image:
                    assert np.array_equal(rgb, np.asarray(image.convert('RGB')))
                checked[name] = {'passed': True, 'source_message_sha256': digest,
                    'native_rgb_exact_match': True, 'shape': list(rgb.shape)}
            else:
                with np.load(root / ref['path'], allow_pickle=False) as cloud:
                    assert np.array_equal(raw_xyz(m), cloud['xyz_sensor_m'], equal_nan=True)
                    assert np.array_equal(point_times(m), cloud['point_timestamp_ns'])
                    fields = field_arrays(m)
                    for key, value in fields.items():
                        assert np.array_equal(value, cloud[key], equal_nan=True)
                checked[name] = {'passed': True, 'source_message_sha256': digest,
                    'xyz_timestamp_additional_fields_exact_match': True,
                    'additional_fields': list(fields)}
            if len(checked) == len(refs):
                break
    assert set(checked) == set(refs), f'Missing original message: {sid}'
    return {'sample_id': row['id'], 'session': row['session'],
        'source_relative_path': item['relative_path'], 'checks': checked}


def run(root, watch):
    items = sorted([i for i in json.loads((root / 'source_inventory.json').read_text())
                    if 'duplicate_of' not in i], key=lambda i: int(i['start_ns']))
    selected = {}
    for item in items:
        session, _ = source_ids(item)
        selected.setdefault(session, item)
    results = []
    for item in selected.values():
        _, sid = source_ids(item)
        while not (root / 'sequences' / sid / 'completed.json').exists():
            if not watch:
                raise FileNotFoundError(sid)
            time.sleep(5)
        result = check_source(root, item)
        results.append(result)
        print(json.dumps(result), flush=True)
    dump(root / 'source_spotcheck.json', {'passed': True, 'sessions': len(results),
        'scope': 'One real frame and its associated LiDAR scan, if present, per session; exact source bytes/arrays. Not every source message.',
        'results': results})


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--dataset', type=Path, required=True)
    p.add_argument('--watch', action='store_true')
    a = p.parse_args()
    run(a.dataset, a.watch)

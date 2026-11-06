#!/usr/bin/env python3
"""Replay STOP ablations against hash-pinned, frozen R50 inference caches.

Rendering reuses the previous video camera/detector panels unchanged. Commands
are recomputed from the original float32 mask crops and source timestamps.
Missing source captures create no output frames, commands or temporal updates.
"""
import argparse
from collections import Counter
from dataclasses import asdict
import json
from pathlib import Path
import subprocess

import av
import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont
import torch

from connectome_policy import DriveReadout
from front_connectome_client import limit_front_command
from r50_front_stop_policy import ARCHITECTURES
from r50_policy_package import load_config, load_policy, sha256
from train_r50_front_stop import evaluate, save


def rows(path):
    return [json.loads(s) for s in path.read_text().splitlines() if s.strip()]


def stop_stats(records):
    stopped = [r for r in records if r['model_command']['reason'] == 'neural_wait']
    runs, current, previous = [], [], None
    for r in records + [None]:
        go = r is None or r['model_command']['reason'] != 'neural_wait'
        continuous = (r is not None and previous is not None
                      and r['recording'] == previous['recording'] and r['time_bin'] == previous['time_bin'] + 1)
        if current and (go or not continuous):
            runs.append(dict(recording=current[0]['recording'], start_sec=current[0]['source_time_sec'],
                             end_sec=current[-1]['source_time_sec'], samples=len(current)))
            current = []
        if not go:
            current.append(r)
        previous = r
    low = [r for r in records if min(r['road_fractions'][1], r['road_fractions'][3]) < .05]
    both = [r for r in records if min(r['road_fractions'][1], r['road_fractions'][3]) >= .05]
    def rate(rr):
        n = sum(r['model_command']['reason'] == 'neural_wait' for r in rr)
        return dict(observations=len(rr), stop=n, stop_rate=n / len(rr) if rr else None)
    return dict(predictions=len(records), stop=len(stopped), stop_rate=len(stopped) / len(records),
        stop_runs=len(runs), single_sample_stop_runs=sum(r['samples'] == 1 for r in runs),
        longest_runs=sorted(runs, key=lambda r: r['samples'], reverse=True)[:20],
        one_or_both_side_road_below_5pct=rate(low), both_sides_at_least_5pct=rate(both),
        empty_front_but_move=sum(r['road_fractions'][0] == 0 and r['model_command']['reason'] != 'neural_wait' for r in records),
        by_split={s: rate([r for r in records if r['split'] == s]) for s in sorted({r['split'] for r in records})},
        by_recording={s: rate([r for r in records if r['recording'] == s]) for s in sorted({r['recording'] for r in records})})


def render(config, selected, predictions):
    out, old = Path(config['validation_output']), Path(config['source_validation'])
    fonts = [ImageFont.truetype('/System/Library/Fonts/AppleSDGothicNeo.ttc', size) for size in (28, 25, 22)]
    videos = []
    previews = out / 'previews'; previews.mkdir(exist_ok=True)
    for recording in ('D20260915', 'D20261104', 'D20261105'):
        rr = rows(old / 'records' / f'{recording}-frames.jsonl')
        actual = [r for r in rr if r['source_present']]
        path = old / 'videos' / f'{recording}-r50-ft-detection-command.mp4'
        destination = out / 'videos' / f'{recording}-{selected}-detection-command.mp4'
        destination.parent.mkdir(exist_ok=True)
        container = av.open(str(path)); stream = container.streams.video[0]
        total = int(stream.frames)
        assert total in (len(rr), len(actual)), (recording, total, len(rr), len(actual))
        timeline = rr if total == len(rr) else actual
        encoder = subprocess.Popen(['ffmpeg', '-y', '-v', 'error', '-f', 'rawvideo', '-pix_fmt', 'rgb24',
            '-s', '1920x1080', '-r', '2.5', '-i', '-', '-an', '-c:v', 'libx264', '-preset', 'veryfast',
            '-crf', '23', '-threads', '4', '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(destination)], stdin=subprocess.PIPE)
        count, previews_saved = 0, 0
        try:
            for index, frame in enumerate(container.decode(stream)):
                row = timeline[index]
                if not row['source_present']:
                    continue
                r = predictions[row['id']]
                assert r['source_pts'] == row['source_pts'] and r['output_frame_index'] == count
                im = Image.fromarray(frame.to_ndarray(format='rgb24'))
                draw = ImageDraw.Draw(im)
                draw.rectangle((0, 0, 1919, 54), fill='#0c1c24')
                draw.text((16, 10), f'R50 v3 고정 검출 + {selected} STOP · 녹화 입력 재계산', font=fonts[0], fill='white')
                sec = r['source_time_sec']
                draw.text((1180, 12), f'{recording} {int(sec)//60:02}:{sec%60:06.3f} · {r["split"]}', font=fonts[1], fill='white')
                draw.rectangle((0, 909, 1919, 1079), fill='#0c1c24')
                command = r['physical_command']
                status = 'STOP' if r['model_command']['reason'] == 'neural_wait' else 'GO'
                draw.text((16, 919), f'새 후보 {status} | vx {command["vx"]:.3f} m/s | yaw {command["yaw_rate"]:+.3f} rad/s | p(stop) {r["stop_probability"]:.4f}', font=fonts[0], fill='#a2e28d')
                draw.text((16, 960), f'기존 v3: {"STOP" if row["model_command"]["reason"] == "neural_wait" else "GO"} · p(stop) {row["stop_probability"]:.4f} | 속도·조향 헤드 가중치 유지', font=fonts[1], fill='white')
                draw.text((16, 1000), '초록 차도 · 갈색 흙 · 노랑 식생 · 주황 연석 · 보라 인도 · 파랑 차량 · 분홍 로봇', font=fonts[2], fill='#cbd4d9')
                draw.text((16, 1036), '영상은 새 명령에 반응하지 않음 · 실제 로봇 제어·완주 검증 아님 · 결손 프레임 생략', font=fonts[2], fill='#f0d38c')
                encoder.stdin.write(np.asarray(im).tobytes())
                if (row['model_command']['reason'] == 'neural_wait' and status == 'GO' and previews_saved < 2
                        and (previews_saved == 0 or sec > 120)):
                    im.save(previews / f'{recording}-changed-{count:05d}.jpg', quality=92)
                    previews_saved += 1
                count += 1
                if count % 500 == 0:
                    print(json.dumps(dict(event='render', recording=recording, frames=count)), flush=True)
        finally:
            encoder.stdin.close(); container.close()
        assert encoder.wait() == 0 and count == len(actual)
        probe = json.loads(subprocess.check_output(['ffprobe', '-v', 'error', '-select_streams', 'v:0',
            '-show_entries', 'stream=nb_frames,r_frame_rate,duration,width,height', '-of', 'json', str(destination)]))['streams'][0]
        assert int(probe['nb_frames']) == len(actual) and probe['r_frame_rate'] == '5/2'
        videos.append(dict(recording=recording, path=str(destination), frames=count,
                           source_bins=len(rr), missing_skipped=len(rr)-len(actual), probe=probe,
                           source_overlay_sha256=sha256(path)))
    concat = out / 'concat.txt'
    concat.write_text(''.join("file 'videos/" + Path(v['path']).name + "'\n" for v in videos))
    combined = out / 'all-three-videos-stop-rework.mp4'
    subprocess.run(['ffmpeg', '-y', '-v', 'error', '-f', 'concat', '-safe', '0', '-i', str(concat),
                    '-c', 'copy', '-movflags', '+faststart', str(combined)], check=True)
    return videos, str(combined)


def validate(config_path, no_video=False):
    torch.set_num_threads(2); cv2.setNumThreads(1)
    c = json.loads(Path(config_path).read_text())
    d, m, out = (Path(c[key]) for key in ('dataset_output', 'model_output', 'validation_output'))
    out.mkdir(exist_ok=True)
    original = rows(d / 'source-observations.jsonl')
    parent_pin = sha256(Path(c['source_package']) / 'model.json')
    source_report = json.loads((Path(c['source_validation']) / 'validation-report.json').read_text())
    assert source_report['model_json_sha256'] == parent_pin
    for cache in sorted((Path(c['source_validation']) / 'masks').glob('*.npz')):
        if cache.name.startswith('.'):
            continue
        with np.load(cache, allow_pickle=False) as z:
            assert str(z['model_json_sha256'].item()) == parent_pin
    with np.load(d / 'frozen-r50-features.npz', allow_pickle=False) as z:
        x, ids = z['x'], z['ids']
    assert all(r['id'] == sid for r, sid in zip(original, ids))
    parent = DriveReadout()
    parent.load_state_dict(torch.load(Path(c['source_package']) / 'policy/drive-readout.pt', map_location='cpu', weights_only=True)['state_dict'])
    parent.eval().requires_grad_(False)
    training = json.loads((m / 'training-report.json').read_text())
    report = dict(schema='r50_360.front_stop_rework.validation.v1', source='frozen R50 v3 masks at original source PTS',
        detector_sha256=c['detector_sha256'], original_v3=stop_stats(original), candidates={},
        selected_front_candidate=training['selected_front_candidate'], selection='development only before full replay',
        source_feature_sha256=sha256(d / 'frozen-r50-features.npz'), source_package_sha256=parent_pin,
        all_frozen_mask_chunks_identify_source_v3_package=True,
        reviewed_decisions_sha256=sha256(d / 'reviewed-decisions.json'),
        actual_robot_commands_sent=False, closed_loop_validated=False, source_missing_policy='skip',
        limitations=['Reviewed GO labels are assistant preferences, not independent human ground truth.',
            'No verified real required-STOP labels in the reviewed set; real missed-stop rate is unknown.',
            'Synthetic blocked-mask tests do not validate real obstacle detection or stopping distance.',
            'Detector and raw velocity/yaw branch are unchanged; improved STOP statistics do not prove steering or safety.',
            'Full replay includes fitting timestamps. Nov4 external/test splits are reported separately.',
            'No default model, Jetson files, threshold or Sidewalk controller was changed.'])
    selected_rows = None
    for architecture in ARCHITECTURES:
        package = m / architecture / 'package'
        pin = sha256(package / 'model.json')
        config = load_config(package, pin, c['detector_sha256'])
        policy = load_policy(package, config)
        records, previous, output_indices = [], None, Counter()
        for index, r in enumerate(original):
            if previous is None or previous['recording'] != r['recording'] or previous['split'] != r['split']:
                policy.reset()
            if previous is not None and (previous['recording'] != r['recording'] or r['time_bin'] - previous['time_bin'] != 1):
                policy.reset()
            command, metrics = policy.infer_pixels(x[index], source_time_sec=r['source_time_sec'])
            physical = limit_front_command(command, .8, .18)
            nr = dict(r, model_command=asdict(command), physical_command=asdict(physical),
                old_v3_command=r['model_command'], old_v3_stop_probability=r['stop_probability'],
                stop_probability=metrics['stop_probability'], policy_metrics=metrics,
                output_frame_index=output_indices[r['recording']], output_time_sec=output_indices[r['recording']] / 2.5,
                commands_published=False)
            assert 0 <= physical.vx <= .8 and abs(physical.yaw_rate) <= .18 and physical.vy == 0
            records.append(nr); output_indices[r['recording']] += 1; previous = r
        statistics = stop_stats(records)
        statistics['model_json_sha256'] = pin
        # Raw action equality is verified even when old/new STOP decisions differ.
        maximum = 0.
        with torch.inference_mode():
            for begin in range(0, len(x), 64):
                tensor = torch.from_numpy(x[begin:begin+64])
                a = parent(tensor)[0]
                b = policy.model(tensor, torch.zeros((len(tensor), 1)))[0]
                maximum = max(maximum, float((a-b).abs().max()))
        statistics['maximum_raw_action_change'] = maximum
        assert maximum == 0.
        # Front STOP is structurally invariant to side/rear perturbations.
        rng = np.random.default_rng(71106)
        subset = x[rng.choice(len(x), 128, replace=False)].copy()
        changed = subset.copy(); changed[:, 1:] = rng.integers(0, 2, changed[:, 1:].shape)
        with torch.inference_mode():
            a = policy.model(torch.from_numpy(subset), torch.zeros((len(subset), 1)))[1].softmax(1)[:, 1]
            b = policy.model(torch.from_numpy(changed), torch.zeros((len(subset), 1)))[1].softmax(1)[:, 1]
        statistics['side_rear_perturbation_maximum_pstop_change'] = float((a-b).abs().max())
        if architecture != 'reset_shared':
            assert statistics['side_rear_perturbation_maximum_pstop_change'] == 0.
        blank = np.zeros((4, 32, 64), np.float32)
        blind_policy = load_policy(package, config)
        blind, blind_metrics = blind_policy.infer_pixels(blank, source_time_sec=0.)
        statistics['blank_input_cold_start'] = dict(stop=blind.reason == 'neural_wait',
                                                   pstop=blind_metrics['stop_probability'])
        with np.load(d / 'synthetic-known-blockage-test.npz', allow_pickle=False) as z:
            statistics['synthetic_heldout_test'] = evaluate(policy.model, z['x'].astype(np.float32), z['y'], z['dt'])
        directory = out / 'records' / architecture; directory.mkdir(parents=True, exist_ok=True)
        for rec in output_indices:
            path = directory / f'{rec}-frames.jsonl'
            path.write_text(''.join(json.dumps(r, ensure_ascii=False, allow_nan=False)+'\n' for r in records if r['recording']==rec))
        report['candidates'][architecture] = statistics
        if architecture == report['selected_front_candidate']:
            selected_rows = {r['id']: r for r in records}
        print(json.dumps(dict(event='policy_replay', architecture=architecture, statistics=statistics)), flush=True)
    save(out / 'validation-report.json', report)
    if not no_video:
        videos, combined = render(c, report['selected_front_candidate'], selected_rows)
        report.update(videos=videos, combined_video=combined,
                      output_frames=sum(v['frames'] for v in videos), missing_source_bins=sum(v['missing_skipped'] for v in videos))
        save(out / 'validation-report.json', report)
    print(json.dumps(dict(event='complete', report=str(out / 'validation-report.json'))), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True)
    parser.add_argument('--no-video', action='store_true')
    args = parser.parse_args()
    validate(args.config, args.no_video)

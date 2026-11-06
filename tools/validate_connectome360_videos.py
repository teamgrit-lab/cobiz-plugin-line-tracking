"""Replay complete recorded 360 videos through the paired detector and actor.

Consumes a source-verified 5 Hz PTS manifest and segment plan. Does not publish
commands, use a route or modify model files. Empty source bins are not inferred.
"""
import argparse
from collections import Counter
from dataclasses import asdict
import json
import math
from pathlib import Path
import shutil
import time

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont
import torch

from connectome360_client import Connectome360Segmenter
from connectome360_runtime import Connectome360Runtime, sha256, unpack_views
from panorama_mask import _panorama_projection
from front_connectome_client import limit_front_command
from render_connectome360_video import Display, clock_text, open_writer, probe
from surround_camera import load_profile, project_panorama


def save(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n')


def illustrate_command(vx, yaw, seconds=4.):
    """A constant-command body-frame illustration, never a planned road path."""
    t = np.linspace(0, seconds, 41)
    if abs(yaw) < 1e-8:
        return np.column_stack((vx*t, np.zeros_like(t)))
    return np.column_stack((vx/yaw*np.sin(yaw*t), vx/yaw*(1-np.cos(yaw*t))))


class DrivingDisplay:
    def __init__(self, model_name, road_sha, policy_sha):
        self.u, self.v, _, self.sectors = _panorama_projection(960, 480, 90)
        font = '/System/Library/Fonts/AppleSDGothicNeo.ttc'
        big = ImageFont.truetype(font, 28)
        normal = ImageFont.truetype(font, 24)
        small = ImageFont.truetype(font, 20)
        image = Image.new('RGB', (1920, 1080), '#111d27')
        draw = ImageDraw.Draw(image)
        draw.text((18, 9), '360° Flyvis · 차도 검출 → 주행·정지 명령 · 그림자 증강 이전 모델', font=big, fill='#edf4f1')
        draw.text((18, 61), '실제 원본 영상', font=normal, fill='#e4ebed')
        draw.text((976, 61), '차도 예측 · 풀·팔 오검출도 그대로 표시', font=normal, fill='#94e6ad')
        for index, name in enumerate(['전방 90°', '좌측 180°', '후방 270°', '우측 0°']):
            draw.text((index*480+12, 600), name, font=normal, fill='#e4ebed')
        draw.text((18, 1012), '초록: 차도 예측 · 음영: 미관측 · 오프라인 명령 계산 · 로봇 제어 없음', font=small, fill='#c9d9d5')
        draw.text((18, 1047), '원본 녹화의 움직임이며 새 명령으로 주행한 결과가 아닙니다.', font=small, fill='#efc88e')
        draw.text((1450, 1014), '주황: 명령 4초 유지 가정', font=small, fill='#efc88e')
        draw.text((1450, 1046), '계산한 궤적 · 실제 경로 아님', font=small, fill='#cbd9dd')
        self.template = cv2.cvtColor(np.asarray(image), cv2.COLOR_RGB2BGR)
        self.identity = f'road {road_sha[:10]} / actor {policy_sha[:10]}'

    def panorama_mask(self, masks):
        output = np.full((480, 960), 255, np.uint8)
        for mask, sector in zip(masks, self.sectors):
            sampled = cv2.remap(mask, self.u, self.v, cv2.INTER_NEAREST, borderMode=cv2.BORDER_CONSTANT)
            output[sector] = sampled[sector]
        return output

    def frame(self, panorama, views, masks, row, raw, capped, stop_probability):
        out = self.template.copy()
        raw_erp = cv2.resize(panorama, (960, 480), interpolation=cv2.INTER_AREA)
        out[104:584, :960] = raw_erp
        out[104:584, 960:] = Display.overlay(raw_erp, self.panorama_mask(masks))
        for k in range(4):
            out[638:908, k*480:(k+1)*480] = Display.overlay(views[k], masks[k])
        cv2.putText(out, f"{row['recording']} {clock_text(row['source_time_sec'])} | {row['split']}",
                    (1010, 35), cv2.FONT_HERSHEY_SIMPLEX, .65, (233, 233, 233), 1, cv2.LINE_AA)
        cv2.putText(out, f'MODEL (10 m/s scale): vx {raw.vx:5.2f} m/s | yaw {raw.yaw_rate:+.3f} rad/s | stop p {stop_probability:.3f}',
                    (20, 941), cv2.FONT_HERSHEY_SIMPLEX, .67, (210, 231, 240), 1, cv2.LINE_AA)
        reason = 'STOP' if capped.reason == 'neural_wait' else 'MOVE'
        turn = 'LEFT' if capped.yaw_rate > .015 else 'RIGHT' if capped.yaw_rate < -.015 else 'STRAIGHT'
        cv2.putText(out, f'JETSON caps (0.50 m/s, 0.18 rad/s): vx {capped.vx:.3f} | yaw {capped.yaw_rate:+.3f} | {reason} {turn if reason=="MOVE" else ""}',
                    (20, 979), cv2.FONT_HERSHEY_SIMPLEX, .67,
                    (120, 222, 155) if reason == 'MOVE' else (100, 170, 255), 1, cv2.LINE_AA)
        # Display-only integration: this never enters either trained model.
        path = illustrate_command(capped.vx, capped.yaw_rate)
        points = np.rint(np.column_stack((1810-path[:, 1]*58, 1058-path[:, 0]*58))).astype(np.int32)
        cv2.rectangle(out, (1707, 919), (1913, 1074), (55, 72, 80), 1)
        for x in [1752, 1810, 1868]: cv2.line(out, (x, 924), (x, 1067), (40, 56, 65), 1)
        for y in [942, 1000, 1058]: cv2.line(out, (1712, y), (1908, y), (40, 56, 65), 1)
        cv2.polylines(out, [points], False, (65, 180, 250), 3, cv2.LINE_AA)
        cv2.circle(out, tuple(points[0]), 4, (235, 240, 240), -1)
        return out

def run_worker(output, worker):
    plan = json.loads((output/'plan.json').read_text())
    manifest = output/'manifest.jsonl'
    if sha256(manifest) != plan['manifest_sha256']:
        raise ValueError('Validation manifest changed')
    rows = [json.loads(line) for line in manifest.read_text().splitlines()]
    jobs = [s for s in plan['segments'] if s['worker'] == worker]
    lookup = {(r['recording'], r['time_bin']): r for r in rows}
    torch.set_num_threads(2)
    cv2.setNumThreads(1)
    model = Connectome360Runtime(plan['model'], plan['model_sha256'], encoder_device='cpu', head_device='cpu')
    adapter = Connectome360Segmenter.__new__(Connectome360Segmenter)
    adapter.details = model.details
    profile = load_profile()
    display = DrivingDisplay(Path(plan['model']).name, plan['road_sha256'], plan['policy_sha256'])
    begun = time.perf_counter()
    completed = []
    all_actual = 0
    planned = sum(j['actual_frames'] for j in jobs)
    for job in jobs:
        name = job['name']
        marker = output/'reports'/(name+'-complete.json')
        if marker.exists():
            record = json.loads(marker.read_text())
            assert record['road_sha256'] == plan['road_sha256'] and record['policy_sha256'] == plan['policy_sha256']
            if record.get('missing_source_policy') != 'skip':
                raise ValueError('Existing segment uses another video timeline; choose a fresh output directory')
            completed.append(record)
            all_actual += record['actual_frames']
            continue
        dest = output/'videos'/(name+'-detection-command.mp4')
        writer = open_writer(dest)
        ids, packed = [], []
        actual = missing = 0
        counters = Counter()
        speeds, yaws, stop_probs, elapsed = [], [], [], []
        preview_bins = {job['start_bin']+offset for offset in [0, 175, 475, 875] if job['start_bin']+offset < job['end_bin']}
        preview_bins.add((job['start_bin']+job['end_bin'])//2)
        preview_bins.add(max(job['start_bin'], job['end_bin']-10))
        saved = set()
        with (output/'records'/(name+'-frames.jsonl')).open('w') as log:
            try:
                for time_bin in range(job['start_bin'], job['end_bin']):
                    row = lookup.get((job['recording'], time_bin))
                    if row is None:
                        missing += 1
                        record = dict(recording=job['recording'], time_bin=time_bin, source_present=False,
                                      detection=None, model_command=None, jetson_capped_command=None,
                                      output_frame_index=None, output_time_sec=None)
                        log.write(json.dumps(record, ensure_ascii=False, allow_nan=False)+'\n')
                        continue
                    else:
                        tick = time.perf_counter()
                        source = Path(row['panorama_absolute'])
                        assert sha256(source) == row['panorama_sha256'], source
                        panorama = cv2.imread(str(source))
                        assert panorama is not None and panorama.shape == (1440, 2880, 3)
                        if row['atlas_absolute']:
                            atlas = Path(row['atlas_absolute'])
                            assert sha256(atlas) == row['atlas_sha256'], atlas
                            views = unpack_views(cv2.imread(str(atlas)))
                        else:
                            views = project_panorama(panorama, profile)
                        t = time.perf_counter()
                        masks, perception = model.road.predict_views(views)
                        raw, policy = model.policy.infer(masks)
                        infer_sec = time.perf_counter()-t
                        transported = adapter.decode_command(asdict(raw))
                        capped = limit_front_command(transported, plan['physical_output_cap_mps'], plan['physical_output_yaw_cap_radps'])
                        assert capped.vy == 0 and 0 <= capped.vx <= .5 and abs(capped.yaw_rate) <= .18
                        assert masks.shape == (4, 270, 480) and np.isin(masks, [0, 1]).all()
                        frame = display.frame(panorama, views, masks, row, raw, capped, policy['stop_probability'])
                        ids.append(row['id'])
                        packed.append(np.packbits(masks.reshape(-1)))
                        stopped = raw.reason == 'neural_wait'
                        counters['learned_stop' if stopped else 'learned_move'] += 1
                        counters['empty_front_mask'] += int(not masks[0].any())
                        counters['empty_front_but_move'] += int(not masks[0].any() and not stopped)
                        counters['all_masks_empty'] += int(not masks.any())
                        counters['all_masks_empty_but_move'] += int(not masks.any() and not stopped)
                        counters['post25_excluded_from_training'] += int(row['split'] == 'excluded_from_training_after_25min')
                        speeds.append(capped.vx); yaws.append(capped.yaw_rate); stop_probs.append(policy['stop_probability']); elapsed.append(infer_sec)
                        record = dict(id=row['id'], recording=row['recording'], source_present=True, time_bin=time_bin,
                            source_pts=row['source_pts'], source_time_sec=row['source_time_sec'], time_base=row['time_base'],
                            output_frame_index=actual, output_time_sec=actual/5.,
                            split=row['split'], source_sha256=row['source_sha256'],
                            source_panorama_sha256=row['panorama_sha256'], input_origin=row['input_origin'],
                            road_fractions=masks.mean(axis=(1, 2)).tolist(), model_command=asdict(raw),
                            transport_capped_command=asdict(transported), jetson_capped_command=asdict(capped),
                            stop_probability=policy['stop_probability'], raw_action=policy['raw_action'],
                            inference_sec=infer_sec, frame_processing_sec=time.perf_counter()-tick,
                            camera_and_inference_age_guard_simulated=False, commands_published=False)
                        for target in sorted(preview_bins):
                            if target <= time_bin < target+10 and target not in saved:
                                cv2.imwrite(str(output/'previews'/f'{name}-{time_bin:06d}.jpg'), frame)
                                saved.add(target)
                        actual += 1; all_actual += 1
                    log.write(json.dumps(record, ensure_ascii=False, allow_nan=False)+'\n')
                    writer.stdin.write(frame.tobytes())
                    if actual and actual % 100 == 0 and row is not None:
                        log.flush()
                        progress = dict(worker=worker, actual_frames=all_actual, planned_frames=planned,
                            recording=job['recording'], source_time_sec=row['source_time_sec'],
                            elapsed_sec=round(time.perf_counter()-begun, 1))
                        save(output/'reports'/f'worker-{worker}-progress.json', progress)
                        print(json.dumps(progress), flush=True)
            finally:
                log.flush()
                writer.stdin.close()
                if writer.wait():
                    raise RuntimeError('Video encoding failed: '+str(dest))
        assert actual == job['actual_frames']
        info = probe(dest)
        source_bins = job['end_bin']-job['start_bin']
        assert actual + missing == source_bins
        expected = actual
        assert int(info['nb_frames']) == expected and info['r_frame_rate'] == '5/1'
        np.savez_compressed(output/'masks'/(name+'-predictions.npz'), ids=np.asarray(ids),
            packed_masks=np.stack(packed), road_sha256=plan['road_sha256'], model_sha256=plan['model_sha256'],
            validation_manifest_sha256=plan['manifest_sha256'])
        stats = dict(segment=job, video=str(dest.relative_to(output)), actual_frames=actual,
            missing_source_bins=missing, source_bins=source_bins,
            missing_source_policy='skip', video_frames=expected, ffprobe=info,
            road_sha256=plan['road_sha256'], policy_sha256=plan['policy_sha256'], counts=dict(counters),
            capped_speed_mean_mps=float(np.mean(speeds)), capped_speed_max_mps=float(np.max(speeds)),
            capped_abs_yaw_mean_radps=float(np.mean(np.abs(yaws))), stop_probability_mean=float(np.mean(stop_probs)),
            mac_cpu_inference_mean_sec=float(np.mean(elapsed)), mac_cpu_inference_p95_sec=float(np.quantile(elapsed, .95)),
            commands_published=False, independent_accuracy_evaluation=False)
        save(marker, stats)
        completed.append(stats)
        print('SEGMENT_COMPLETE', name, actual, flush=True)
    save(output/'reports'/f'worker-{worker}-complete.json', dict(worker=worker, segments=completed,
        actual_frames=all_actual, elapsed_sec=time.perf_counter()-begun))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--worker', choices=['a', 'b'], required=True)
    args = p.parse_args()
    run_worker(args.output.resolve(), args.worker)


if __name__ == '__main__':
    main()

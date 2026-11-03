"""Render raw Flyvis road predictions on timestamped saved real 360 video frames.

This is offline detection replay, without driving commands or accuracy claims.
All predicted road pixels are retained. Missing source bins get gap cards.
"""
import argparse
from collections import defaultdict
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
from zoneinfo import ZoneInfo

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont
import torch


def digest(path):
    with Path(path).open('rb') as handle:
        return hashlib.file_digest(handle, 'sha256').hexdigest()


def write_json(path, data):
    Path(path).write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False) + '\n')


def clock_text(stamp):
    return f'{int(stamp)//60:02}:{stamp%60:06.3f}'


class Display:
    width, height = 1920, 1080

    def __init__(self, projection, model_name, road_sha):
        self.u, self.v, self.visible, self.sectors = projection(960, 480, 90)
        font_path = '/System/Library/Fonts/AppleSDGothicNeo.ttc'
        large = ImageFont.truetype(font_path, 31)
        medium = ImageFont.truetype(font_path, 24)
        small = ImageFont.truetype(font_path, 22)
        image = Image.new('RGB', (self.width, self.height), '#111d27')
        draw = ImageDraw.Draw(image)
        draw.text((22, 12), '360° 실영상 · 이번에 학습한 커넥톰 차도 검출', font=large, fill='#f0f5f4')
        draw.text((22, 62), '실제 입력 영상', font=medium, fill='#e2e9e9')
        draw.text((980, 62), '차도 예측 오버레이 · 네 방향 합성', font=medium, fill='#7ff2a6')
        for j, name in enumerate(['전방 90°', '좌측 180°', '후방 270°', '우측 0°']):
            draw.text((j*480+14, 598), name, font=medium, fill='#e2e9e9')
        draw.text((22, 930), '초록: 차도 예측  |  상·하단 음영: 네 투영 영상 밖의 미관측 영역', font=medium, fill='#9fe2ba')
        draw.text((22, 972), '정답·정확도 평가가 아닙니다. 풀·팔의 오검출도 제거하지 않고 표시합니다.', font=small, fill='#efcc92')
        draw.text((22, 1010), '검출만 실행 · 주행 제어 없음 · 합성 검출 누락/마스크 후처리 없음', font=small, fill='#b7c4cc')
        draw.text((22, 1045), model_name + ' | road SHA-256: ' + road_sha[:16], font=small, fill='#a3b6c1')
        self.template = cv2.cvtColor(np.asarray(image), cv2.COLOR_RGB2BGR)

    def panorama_mask(self, masks):
        labels = np.full((480, 960), 255, np.uint8)
        for mask, sector in zip(masks, self.sectors):
            sampled = cv2.remap(mask, self.u, self.v, cv2.INTER_NEAREST,
                                borderMode=cv2.BORDER_CONSTANT)
            labels[sector] = sampled[sector]
        return labels

    @staticmethod
    def overlay(image, mask):
        result = image.copy()
        selected = mask == 1
        result[selected] = (.56 * result[selected] + .44 * np.array([70, 225, 70])).astype(np.uint8)
        result[mask == 255] = (result[mask == 255] * .48).astype(np.uint8)
        return result

    def frame(self, panorama, views, masks, row):
        canvas = self.template.copy()
        raw = cv2.resize(panorama, (960, 480), interpolation=cv2.INTER_AREA)
        canvas[104:584, :960] = raw
        canvas[104:584, 960:] = self.overlay(raw, self.panorama_mask(masks))
        for j in range(4):
            canvas[638:908, j*480:(j+1)*480] = self.overlay(views[j], masks[j])
        cv2.putText(canvas, f"{row['recording']}  {clock_text(row['source_time_sec'])}  {row['split']}  |  5 FPS actual PTS",
                    (980, 44), cv2.FONT_HERSHEY_SIMPLEX, .7, (235, 235, 235), 1, cv2.LINE_AA)
        return canvas

    def gap(self, recording, stamp):
        result = self.template.copy()
        cv2.putText(result, f'{recording}  {clock_text(stamp)}', (610, 300),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, (230, 230, 230), 2, cv2.LINE_AA)
        cv2.putText(result, 'SOURCE FRAME MISSING - NO INFERENCE', (370, 420),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (135, 190, 240), 2, cv2.LINE_AA)
        return result


def open_writer(path):
    return subprocess.Popen(['ffmpeg', '-y', '-nostdin', '-hide_banner', '-loglevel', 'error',
        '-f', 'rawvideo', '-pix_fmt', 'bgr24', '-s', '1920x1080', '-r', '5', '-i', 'pipe:0',
        '-an', '-c:v', 'libx264', '-preset', 'veryfast', '-crf', '22', '-threads', '2',
        '-g', '10', '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(path)], stdin=subprocess.PIPE)


def probe(path):
    return json.loads(subprocess.check_output(['ffprobe','-v','error','-select_streams','v:0',
        '-show_entries','stream=codec_name,width,height,nb_frames,r_frame_rate,duration',
        '-of','json',str(path)]))['streams'][0]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', type=Path, required=True)
    parser.add_argument('--dataset', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--mode', choices=['preview', 'full'], default='preview')
    parser.add_argument('--recordings', nargs='+', help='Optional recording names for independent full-video workers')
    parser.add_argument('--reuse-cache', type=Path)
    parser.add_argument('--device', choices=['cpu', 'mps'], default='mps')
    args = parser.parse_args()
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    (out/'previews').mkdir()
    sys.path.insert(0, str(args.model/'runtime/tools'))
    from connectome360_dataset import four_views
    from connectome_flow import FlyvisFlow
    from connectome_road import ConnectomeRoadPerception
    from dangjin_camera import _panorama_projection
    from surround_camera import load_profile, profile_sha256

    torch.set_num_threads(2)
    cv2.setNumThreads(1)
    config = json.loads((args.model/'model.json').read_text())
    for name, expected in [('flyvis_encoder.pt', config['encoder_sha256']),
                           ('road/road-readout.pt', config['road_sha256'])]:
        assert digest(args.model/name) == expected, name
    manifest_sha = digest(args.dataset/'frames.jsonl')
    profile = load_profile(args.model/'runtime/environments/dangjin/camera360-profile.json')
    assert profile_sha256(profile) == config['camera_profile_sha256']
    rows = [json.loads(line) for line in (args.dataset/'frames.jsonl').read_text().splitlines()]
    assert all(r['recording'] != 'ID482' or r['source_time_sec'] < 1500 for r in rows)
    frame_index = {r['id']: r for r in rows}
    assert len(frame_index) == len(rows)
    segments = []
    if args.mode == 'preview':
        for clip, start, end in [('ID482',56,86),('ID482',435,465),('RSSP00',42,72),('RSSP06',50,80)]:
            selected = [r for r in rows if r['recording'] == clip and start <= r['source_time_sec'] < end]
            assert selected, (clip, start, end)
            segments.append(dict(name=f'{clip}-{start:04}-{end:04}', recording=clip,
                                 start_bin=start*5, end_bin=end*5, rows=selected))
    else:
        groups = defaultdict(list)
        for r in rows:
            if not args.recordings or r['recording'] in args.recordings:
                groups[r['recording']].append(r)
        if args.recordings:
            assert set(groups) == set(args.recordings), 'Unknown recording selection'
        for clip, selected in groups.items():
            segments.append(dict(name=clip, recording=clip, start_bin=0,
                                 end_bin=max(r['time_bin'] for r in selected)+1, rows=selected))
    plan = dict(model=str(args.model.resolve()), dataset=str(args.dataset.resolve()),
                detector_sha256=config['road_sha256'], encoder_sha256=config['encoder_sha256'],
                dataset_manifest_sha256=manifest_sha, camera_profile_sha256=config['camera_profile_sha256'],
                sampling_hz=5, mode=args.mode, road_threshold=float(json.loads((args.model/'road/simulation-training.json').read_text())['threshold']),
                input_representation='saved actual decoded JPEG panoramas and their exact four-view PNG projections',
                all_predicted_road_pixels_preserved=True, synthetic_detection_faults=False,
                driving_commands=False, accuracy_measured=False,
                segments=[{k:v for k,v in s.items() if k!='rows'}|{'actual_frames':len(s['rows'])} for s in segments])
    write_json(out/'plan.json', plan)
    write_json(out/'process.json', dict(pid=os.getpid(), started_at=datetime.now(ZoneInfo('Asia/Seoul')).isoformat()))
    reused = {}
    if args.reuse_cache:
        with np.load(args.reuse_cache) as cached:
            assert str(cached['road_sha256']) == config['road_sha256']
            assert str(cached['manifest_sha256']) == manifest_sha
            reused = dict(zip(cached['ids'].astype(str), cached['packed_masks']))
    encoder = FlyvisFlow(args.model/'flyvis_encoder.pt', config['encoder_sha256'], 'cpu')
    detector = ConnectomeRoadPerception(encoder, args.model/'road', args.device)
    display = Display(_panorama_projection, args.model.name, config['road_sha256'])
    started = time.monotonic()
    cache_ids, cache_masks = [], []
    clips = []
    total_actual = total_missing = inferred = reuse_count = 0
    for segment in segments:
        current = {r['time_bin']: r for r in segment['rows']}
        assert len(current) == len(segment['rows'])
        dest = out/(segment['name']+'-overlay.mp4')
        writer = open_writer(dest)
        actual = missing = 0
        midpoint = len(segment['rows'])//2
        with (out/(segment['name']+'-frames.jsonl')).open('w') as log:
            try:
                for time_bin in range(segment['start_bin'], segment['end_bin']):
                    row = current.get(time_bin)
                    if row is None:
                        frame = display.gap(segment['recording'], time_bin/5.)
                        missing += 1
                    else:
                        panorama = cv2.imread(str(args.dataset/row['panorama']))
                        views = four_views(args.dataset, row['views_atlas'])
                        assert panorama is not None and panorama.shape == (1440,2880,3)
                        assert digest(args.dataset/row['panorama']) == row['panorama_sha256']
                        assert digest(args.dataset/row['views_atlas']) == row['atlas_sha256']
                        tick = time.monotonic()
                        if row['id'] in reused:
                            packed = reused[row['id']]
                            masks = np.unpackbits(packed, count=4*270*480).reshape(4,270,480)
                            reuse_count += 1
                        else:
                            masks, metrics = detector.predict_views(list(views))
                            packed = np.packbits(masks.reshape(-1))
                            inferred += 1
                        assert masks.shape == (4,270,480) and np.isin(masks, [0,1]).all()
                        cache_ids.append(row['id'])
                        cache_masks.append(packed)
                        frame = display.frame(panorama, views, masks, row)
                        log.write(json.dumps(dict(id=row['id'],recording=row['recording'],source_pts=row['source_pts'],
                            source_time_sec=row['source_time_sec'],split=row['split'],time_bin=time_bin,
                            road_fractions=masks.mean(axis=(1,2)).tolist(),
                            cached_prediction=row['id'] in reused, wall_ms=(time.monotonic()-tick)*1000))+'\n')
                        if actual in {0, midpoint}:
                            cv2.imwrite(str(out/'previews'/(segment['name']+f'-{actual:04d}.jpg')), frame)
                        actual += 1
                        total_actual += 1
                        if total_actual % 100 == 0:
                            print(json.dumps(dict(actual_frames=total_actual,planned=sum(len(s['rows']) for s in segments),
                                recording=row['recording'],source_time_sec=row['source_time_sec'],
                                elapsed_sec=round(time.monotonic()-started,1))),flush=True)
                    writer.stdin.write(frame.tobytes())
            finally:
                writer.stdin.close()
                result = writer.wait()
                if result:raise RuntimeError(f'Video encoder failed: {dest}')
        expected = segment['end_bin']-segment['start_bin']
        video = probe(dest)
        assert video['codec_name']=='h264' and int(video['nb_frames'])==expected
        assert (int(video['width']),int(video['height']))==(1920,1080)
        total_missing += missing
        clips.append(dict(recording=segment['recording'],file=dest.name,actual_frames=actual,
                          missing_source_bins=missing,video_frames=expected,duration_sec=expected/5.,ffprobe=video))
        write_json(out/'progress.json',dict(completed_clips=clips,actual_frames=total_actual,
                                          elapsed_sec=time.monotonic()-started))
        print('CLIP_COMPLETE',json.dumps(clips[-1]),flush=True)
    np.savez_compressed(out/'predictions.npz',ids=np.asarray(cache_ids),packed_masks=np.stack(cache_masks),
                        road_sha256=config['road_sha256'],manifest_sha256=manifest_sha)
    concat = out/'concat.txt'
    concat.write_text(''.join("file '"+c['file'].replace("'", "'\\''")+"'\n" for c in clips))
    combined = out/('connectome360-'+args.mode+'-all-overlay.mp4')
    subprocess.run(['ffmpeg','-y','-nostdin','-hide_banner','-loglevel','error','-f','concat','-safe','0',
                    '-i',str(concat),'-c','copy','-movflags','+faststart',str(combined)],check=True)
    final = probe(combined)
    assert int(final['nb_frames']) == sum(c['video_frames'] for c in clips)
    report = dict(plan=plan,actual_frames=total_actual,missing_source_bins=total_missing,
                  new_inferences=inferred,reused_inferences=reuse_count,clips=clips,
                  combined_video=combined.name,combined_ffprobe=final,
                  elapsed_sec=time.monotonic()-started,completed_at=datetime.now(ZoneInfo('Asia/Seoul')).isoformat())
    write_json(out/'complete.json',report)
    shutil.copy2(__file__,out/Path(__file__).name)
    (out/'README.md').write_text(
        '# 이번 360도 커넥톰 모델의 실영상 차도 검출 오버레이\n\n'
        f'모델: {args.model.name}\n검출기 SHA-256: {config["road_sha256"]}\n\n'
        '위 왼쪽은 실제 파노라마, 위 오른쪽은 네 방향 예측을 파노라마에 합성한 결과다. 아래는 전방 90도·좌측 180도·후방 270도·우측 0도의 실제 모델 입력과 예측이다.\n\n'
        '네 방향 화면의 초록은 각 입력에서 모델이 차도로 예측한 모든 픽셀이다. 파노라마 합성은 각 위치와 가장 가까운 방향의 예측을 사용하며 중첩 구간의 예측을 합집합으로 늘리지 않는다. 풀과 팔의 오검출도 지우지 않았다. 비차도는 원본 색을 유지한다. 상·하단 음영은 투영 영상 밖이라 추론하지 않은 영역이다. 모델은 차도/비차도 이진 출력이므로 식생·연석 등 다른 클래스를 따로 색칠하지 않는다.\n\n'
        '실제 원본에서 저장해 둔 5 FPS JPEG 파노라마와 동일 시점의 PNG 네 방향 영상을 사용했다. 프레임 파일 해시를 다시 확인했다. ID482는 25:00 미만만 포함한다. 누락된 원본 시간 구간에는 예측을 복제하지 않고 SOURCE FRAME MISSING 카드를 표시한다. 구간별 원본 시간과 train/development/validation/test/gap 분할을 영상과 로그에 표시한다.\n\n'
        '이번 결과는 녹화 영상의 검출 시각화다. 학습에 사용한 구간을 포함하므로 독립적인 일반화 정확도 평가가 아니다. 주행 헤드·합성 검출 누락·실제 로봇 제어는 실행하지 않았다.\n\n'
        f'합친 영상: {combined.name}\n실제 추론 시점: {total_actual}\n원본 누락 카드: {total_missing}\n'
        '세부 원본 PTS와 차도 예측 면적은 각 frames.jsonl, 원래 4×270×480 예측은 predictions.npz에 보존했다.\n')
    print(json.dumps(dict(output=str(out),video=str(combined),actual_frames=total_actual,
                          missing_source_bins=total_missing,duration_sec=final['duration']),ensure_ascii=False),flush=True)


if __name__=='__main__':main()

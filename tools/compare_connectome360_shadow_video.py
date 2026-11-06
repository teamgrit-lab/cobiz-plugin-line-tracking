"""Compare baseline and shadow-finetuned 360 road/drive heads on the same PTS.

Only original real images are inferred. Cached baseline masks are accepted only
when their model and dataset hashes match. Predicted commands are logged but
never published to a robot or simulation.
"""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
from types import SimpleNamespace
from zoneinfo import ZoneInfo

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont
import torch

from render_connectome360_video import Display, open_writer, probe, clock_text


def digest(path):
    with Path(path).open('rb') as handle:
        return hashlib.file_digest(handle,'sha256').hexdigest()


def save(path,data):
    path.write_text(json.dumps(data,ensure_ascii=False,indent=2,allow_nan=False)+'\n')


class Comparison:
    def __init__(self,projection,old_sha,new_sha,old_threshold,new_threshold):
        self.u,self.v,_,self.sectors=projection(640,320,90)
        font='/System/Library/Fonts/AppleSDGothicNeo.ttc'
        large=ImageFont.truetype(font,29);normal=ImageFont.truetype(font,24)
        small=ImageFont.truetype(font,20)
        image=Image.new('RGB',(1920,1080),'#111d27');draw=ImageDraw.Draw(image)
        draw.text((15,9),'360° 커넥톰 · 그림자 증강 전후 비교 · 같은 원본/시각 · 5 FPS',font=large,fill='#edf4f1')
        for k,title in enumerate(['실제 원본',f'기존 모델 · 임계값 {old_threshold:.2f}',f'그림자 증강 · 임계값 {new_threshold:.2f}']):
            x=k*640
            draw.text((x+12,54),title,font=normal,fill='#f2cd92' if k==1 else '#a1edbb')
            draw.text((x+12,426),'전방 90°',font=normal,fill='#e6eded')
            for j,name in enumerate(['좌측 180°','후방 270°','우측 0°']):
                draw.text((x+j*212+6,830),name,font=small,fill='#dce5e7')
        draw.text((14,980),'초록: 차도 예측 · 음영: 미관측',font=small,fill='#96e3af')
        draw.text((14,1013),'오검출을 제거하지 않은 원래 출력',font=small,fill='#edcf9b')
        draw.text((14,1045),'속도·회전은 오프라인 예측 · 로봇 제어 없음',font=small,fill='#b4c3cc')
        draw.text((650,1045),'이전 검출 해시: '+old_sha[:16],font=small,fill='#c5cdd1')
        draw.text((1290,1045),'신규 검출 해시: '+new_sha[:16],font=small,fill='#c5cdd1')
        self.template=cv2.cvtColor(np.asarray(image),cv2.COLOR_RGB2BGR)

    def panorama(self,masks):
        result=np.full((320,640),255,np.uint8)
        for mask,sector in zip(masks,self.sectors):
            sampled=cv2.remap(mask,self.u,self.v,cv2.INTER_NEAREST,borderMode=cv2.BORDER_CONSTANT)
            result[sector]=sampled[sector]
        return result

    def frame(self,pano,views,old,new,row,old_command,new_command,old_policy,new_policy):
        canvas=self.template.copy()
        raw=cv2.resize(pano,(640,320),interpolation=cv2.INTER_AREA)
        for col,masks in enumerate([None,old,new]):
            x=col*640
            canvas[100:420,x:x+640]=raw if masks is None else Display.overlay(raw,self.panorama(masks))
            front=views[0] if masks is None else Display.overlay(views[0],masks[0])
            canvas[462:822,x:x+640]=cv2.resize(front,(640,360),interpolation=cv2.INTER_AREA)
            for j,sector in enumerate([1,2,3]):
                view=views[sector] if masks is None else Display.overlay(views[sector],masks[sector])
                canvas[857:973,x+j*212:x+j*212+206]=cv2.resize(view,(206,116),interpolation=cv2.INTER_AREA)
        cv2.putText(canvas,f"{row['recording']}  {clock_text(row['source_time_sec'])}  {row['split']}",
                    (1050,33),cv2.FONT_HERSHEY_SIMPLEX,.66,(240,240,240),1,cv2.LINE_AA)
        for x,c,p in [(650,old_command,old_policy),(1290,new_command,new_policy)]:
            cv2.putText(canvas,f'vx {c.vx:.2f} m/s   yaw {c.yaw_rate:+.3f} rad/s',
                        (x,995),cv2.FONT_HERSHEY_SIMPLEX,.56,(225,232,235),1,cv2.LINE_AA)
            cv2.putText(canvas,f'{c.reason}   stop probability {p["stop_probability"]:.3f}',
                        (x,1025),cv2.FONT_HERSHEY_SIMPLEX,.52,(225,232,235),1,cv2.LINE_AA)
        return canvas

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--before',type=Path,required=True)
    parser.add_argument('--after',type=Path,required=True)
    parser.add_argument('--baseline-preview',type=Path,required=True)
    parser.add_argument('--dataset',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();out=args.output.resolve();out.mkdir(parents=True,exist_ok=False)
    (out/'previews').mkdir()
    sys.path.insert(0,str(args.after/'runtime/tools'))
    from connectome360_dataset import four_views
    from connectome_flow import FlyvisFlow
    from connectome_road import ConnectomeRoadPerception
    from connectome_policy import ConnectomeDrivePolicy
    from panorama_mask import _panorama_projection
    before=json.loads((args.before/'model.json').read_text())
    after=json.loads((args.after/'model.json').read_text())
    assert before['encoder_sha256']==after['encoder_sha256']
    for root,config in [(args.before,before),(args.after,after)]:
        assert digest(root/'road/road-readout.pt')==config['road_sha256']
        assert digest(root/'policy/drive-readout.pt')==config['policy_sha256']
    manifest_sha=digest(args.dataset/'frames.jsonl')
    with np.load(args.baseline_preview/'predictions.npz') as cache:
        assert str(cache['road_sha256'])==before['road_sha256']
        assert str(cache['manifest_sha256'])==manifest_sha
        old_masks=dict(zip(cache['ids'].astype(str),cache['packed_masks']))
    original_report=json.loads((args.baseline_preview/'complete.json').read_text())
    segments=original_report['plan']['segments']
    rows=[json.loads(x) for x in (args.dataset/'frames.jsonl').read_text().splitlines()]
    lookup={(r['recording'],r['time_bin']):r for r in rows}
    encoder=FlyvisFlow(args.after/'flyvis_encoder.pt',after['encoder_sha256'],'cpu')
    detector=ConnectomeRoadPerception(encoder,args.after/'road','mps')
    old_actor=ConnectomeDrivePolicy(args.before/'policy',SimpleNamespace(details={
        'weights_sha256':before['road_sha256'],'encoder_sha256':before['encoder_sha256']}),'cpu',max_speed_mps=10.)
    new_actor=ConnectomeDrivePolicy(args.after/'policy',detector,'cpu',max_speed_mps=10.)
    thresholds=[json.loads((root/'road/simulation-training.json').read_text())['threshold'] for root in [args.before,args.after]]
    display=Comparison(_panorama_projection,before['road_sha256'],after['road_sha256'],*thresholds)
    single_display=Display(_panorama_projection,args.after.name+' shadow',after['road_sha256'])
    comparison=out/'before-after-shadow-2min-5fps.mp4'
    single=out/'shadow-augmented-2min-5fps-overlay.mp4'
    writers=[open_writer(comparison),open_writer(single)]
    log=(out/'frames.jsonl').open('w')
    ids,packed=[],[];actual=missing=index=0
    counts={'old_stop':0,'new_stop':0};saved=set()
    torch.set_num_threads(2);cv2.setNumThreads(1)
    try:
        for segment in segments:
            for time_bin in range(segment['start_bin'],segment['end_bin']):
                row=lookup.get((segment['recording'],time_bin))
                if row is None:
                    missing+=1
                    log.write(json.dumps(dict(recording=segment['recording'],time_bin=time_bin,
                        source_present=False,output_frame_index=None,output_time_sec=None))+'\n')
                    continue
                else:
                    assert row['id'] in old_masks
                    assert digest(args.dataset/row['panorama'])==row['panorama_sha256']
                    assert digest(args.dataset/row['views_atlas'])==row['atlas_sha256']
                    pano=cv2.imread(str(args.dataset/row['panorama']))
                    views=four_views(args.dataset,row['views_atlas'])
                    old=np.unpackbits(old_masks[row['id']],count=4*270*480).reshape(4,270,480)
                    new,metrics=detector.predict_views(list(views))
                    c0,p0=old_actor.infer(old);c1,p1=new_actor.infer(new)
                    frames=[display.frame(pano,views,old,new,row,c0,c1,p0,p1),single_display.frame(pano,views,new,row)]
                    ids.append(row['id']);packed.append(np.packbits(new.reshape(-1)))
                    counts['old_stop']+=c0.reason=='neural_wait';counts['new_stop']+=c1.reason=='neural_wait'
                    def command(c,p):return dict(vx=c.vx,vy=c.vy,yaw_rate=c.yaw_rate,reason=c.reason,stop_probability=p['stop_probability'])
                    log.write(json.dumps(dict(id=row['id'],recording=row['recording'],source_time_sec=row['source_time_sec'],
                        source_present=True,output_frame_index=index,output_time_sec=index/5.,
                        source_pts=row['source_pts'],split=row['split'],old=command(c0,p0),new=command(c1,p1),
                        old_road_fractions=old.mean(axis=(1,2)).tolist(),new_road_fractions=new.mean(axis=(1,2)).tolist(),
                        mask_disagreement=float((old!=new).mean())))+'\n')
                    for sample in [0,175,325,475]:
                        if sample<=index<sample+5 and sample not in saved:
                            cv2.imwrite(str(out/'previews'/f'comparison-{sample:04d}.jpg'),frames[0]);saved.add(sample)
                    actual+=1
                    if actual%100==0:print('COMPARISON',actual,'/',len(old_masks),flush=True)
                for writer,frame in zip(writers,frames):writer.stdin.write(frame.tobytes())
                index+=1
    finally:
        log.close()
        for writer in writers:
            writer.stdin.close()
            if writer.wait():raise RuntimeError('Comparison video encoding failed')
    source_bins=sum(segment['end_bin']-segment['start_bin'] for segment in segments)
    assert actual==len(old_masks)==index and actual+missing==source_bins
    np.savez_compressed(out/'after-predictions.npz',ids=np.asarray(ids),packed_masks=np.stack(packed),
        road_sha256=after['road_sha256'],manifest_sha256=manifest_sha)
    shutil.copy2(args.baseline_preview/'predictions.npz',out/'before-predictions.npz')
    videos={p.name:probe(p) for p in [comparison,single]}
    for info in videos.values():
        assert info['r_frame_rate']=='5/1' and int(info['nb_frames'])==actual
        assert abs(float(info['duration'])-actual/5.)<1e-3
    report=dict(before_model=str(args.before),after_model=str(args.after),before_hashes=before,after_hashes=after,
        same_original_input_pts=True,same_segments=segments,dataset_manifest_sha256=manifest_sha,
        actual_frames=actual,source_missing_bins=missing,source_bins=source_bins,
        missing_source_policy='skip',video_frames=index,videos=videos,
        actor_command_counts=counts,before_threshold=thresholds[0],after_threshold=thresholds[1],accuracy_measured=False,commands_published=False,
        completed_at=datetime.now(ZoneInfo('Asia/Seoul')).isoformat())
    save(out/'complete.json',report);shutil.copy2(__file__,out/Path(__file__).name)
    (out/'README.md').write_text('''# 그림자 증강 전후 — 같은 대표 2분, 5 FPS

before-after-shadow-2min-5fps.mp4는 원본 / 기존 모델 / 그림자 증강 재학습 모델을 나란히 비교한다. 위쪽은 360도 파노라마, 중앙은 전방 90도, 아래는 좌측·후방·우측이다. 초록은 차도 예측이며 오검출을 제거하지 않았다. 상하단 음영은 미관측이다.

각 모델의 속도·회전·정지 확률은 해당 모델의 검출 마스크를 주행 헤드에 넣어 계산한 오프라인 예측이다. 로봇이나 시뮬레이션에 명령을 발행하지 않았다. 영상은 원본 녹화의 움직임이며 새 명령에 반응한 주행 검증이 아니다.

표시한 속도는 최대 10 m/s인 시뮬레이션 출력 척도이며 녹화 당시의 실제 속도나 Jetson 실기 상한을 뜻하지 않는다.

각 열에 표시한 임계값은 실제 모델에 적용한 값이다. 새 모델의 임계값은 ID482 개발 22장의 원본/합성 그림자 예측으로 선택했다. 이 대표 영상이나 RSSP 평가 라벨은 임계값 선택에 사용하지 않았다.

원본 구간과 실제 입력 시점은 기존 비교 대상과 동일하다. 원본 프레임이 없는 구간은 건너뛰고 실제 프레임만 연결하므로 출력 영상은 원본 구간 합계보다 짧다. 화면에는 원본 시각을 유지하고 frames.jsonl의 output_frame_index/output_time_sec로 출력 재생 위치를 기록한다. 이번 비교 입력에 합성 그림자나 검출 누락을 추가하지 않았다. 증강은 학습에서만 사용했다.

shadow-augmented-2min-5fps-overlay.mp4는 새 검출기만 표시한 영상이다. complete.json에 모델 해시와 구간, frames.jsonl에 같은 PTS별 명령·차도 면적, before/after-predictions.npz에 원래 예측 마스크를 보존했다. 실제 영상 구간은 학습에 사용한 부분을 포함하므로 독립적인 정확도 평가는 아니다.
''')
    print(json.dumps(dict(output=str(out),comparison=str(comparison),actual_frames=actual),ensure_ascii=False),flush=True)


if __name__=='__main__':main()

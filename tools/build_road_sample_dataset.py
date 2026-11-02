#!/usr/bin/env python3
"""Read-only, bounded MCAP extraction for a reviewable real-world pilot dataset.

Run with .venv/bin/python. Source messages remain separate from estimated labels.
Point time is preserved; scans are associated by acquisition midpoint, not arrival.
"""
import argparse
import hashlib
import json
import subprocess
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory

from lidar_common import a2_livox_front_transforms, stamp_ns
from rosbag_mcap_to_mp4 import decode_video_frame

CAMERA='/a2/front_camera/image_raw'
LIDAR='/livox/lidar'
INFO='/a2/front_camera/camera_info'
IMU='/livox/imu'
GPS='/ublox_gps_node/fix_velocity'
EPISODES=[
    {'id':'straight','part':'1-009','offset_s':10.,'duration_s':12.},
    {'id':'right_turn','part':'7-027','offset_s':9.,'duration_s':12.},
    {'id':'vehicle','part':'32-025','offset_s':12.,'duration_s':16.},
]

def write_json(path,data):
    path.write_text(json.dumps(data,ensure_ascii=False,indent=2,allow_nan=False)+'\n')

def point_times(message):
    field=next((f for f in message.fields if f.name=='timestamp'),None)
    if field is None or field.datatype!=8:raise ValueError('Recorded float64 point timestamp required')
    dtype=np.dtype({'names':['t'],'formats':['>f8' if message.is_bigendian else '<f8'],
                    'offsets':[field.offset],'itemsize':message.point_step})
    values=np.ndarray((message.height,message.width),dtype=dtype,buffer=bytes(message.data),
        strides=(message.row_step,message.point_step))['t'].reshape(-1).copy()
    if not np.isfinite(values).all():raise ValueError('nonfinite point timestamps')
    # This source stores absolute Unix nanoseconds as float64. Do not silently
    # infer a unit for another source; require consistency with the header.
    header=stamp_ns(message.header)
    if abs(float(np.min(values))-header)>2e8 or np.ptp(values)>2e8:
        raise ValueError('point timestamp units or header mismatch')
    return values

def metadata(entry):
    m=entry.decoded_message
    return {'topic':entry.channel.topic,'log_time_ns':str(entry.message.log_time),
        'header_time_ns':str(stamp_ns(m.header)),'frame_id':m.header.frame_id,
        'message_sha256':hashlib.sha256(entry.message.data).hexdigest()}

def raw_xyz(message):
    """Preserve zero/invalid source points so per-point times stay aligned."""
    fields={f.name:f for f in message.fields};endian='>' if message.is_bigendian else '<'
    dtype=np.dtype({'names':['x','y','z'],
        'formats':[endian+{7:'f4',8:'f8'}[fields[k].datatype] for k in ['x','y','z']],
        'offsets':[fields[k].offset for k in ['x','y','z']],'itemsize':message.point_step})
    raw=np.ndarray((message.height,message.width),dtype=dtype,buffer=bytes(message.data),
        strides=(message.row_step,message.point_step))
    return np.column_stack([raw[k].reshape(-1) for k in ['x','y','z']])

def extract(source,output):
    output.mkdir(parents=True,exist_ok=True);rows=[];inventories=[];clips=[]
    for ep in EPISODES:
        path=source/f'a2_record_default_20260915_153503_{ep["part"]}.mcap'
        directory=output/ep['id'];directory.mkdir(exist_ok=True)
        for name in ['images','lidar','review']: (directory/name).mkdir(exist_ok=True)
        with path.open('rb') as stream:
            reader=make_reader(stream,decoder_factories=[DecoderFactory()]);summary=reader.get_summary()
            stats=summary.statistics;start=stats.message_start_time+round(ep['offset_s']*1e9);end=start+round(ep['duration_s']*1e9)
            topics={ch.topic:{'type':summary.schemas[ch.schema_id].name,'messages':stats.channel_message_counts.get(cid,0)} for cid,ch in summary.channels.items()}
            source_info={'path':str(path),'file':path.name,'bytes':path.stat().st_size,'mtime_ns':str(path.stat().st_mtime_ns),
                'bag_start_ns':str(stats.message_start_time),'bag_end_ns':str(stats.message_end_time),'topics':topics,
                'command_topics_present':[t for t in topics if any(k in t.lower() for k in ['cmd_vel','sport/request','command','control'])]}
            inventories.append(source_info)
            camera=[];scans=[];imus=[];gps=[];info=None;next_sample=start;next_video=start;video_index=[]
            video=subprocess.Popen(['ffmpeg','-loglevel','error','-y','-f','rawvideo','-pixel_format','rgb24',
                '-video_size','640x360','-framerate','10','-i','-','-an','-c:v','libx264','-crf','21',
                '-pix_fmt','yuv420p','-movflags','+faststart',str(directory/'context.mp4')],stdin=subprocess.PIPE)
            try:
                for entry in reader.iter_decoded_messages(topics=[CAMERA,LIDAR,INFO,IMU,GPS],start_time=start-500_000_000,end_time=end+500_000_000):
                    topic=entry.channel.topic;m=entry.decoded_message;header=stamp_ns(m.header)
                    if topic==INFO and info is None:
                        info={**metadata(entry),'width':m.width,'height':m.height,'K':list(m.k),'D':list(m.d),
                            'R':list(m.r),'P':list(m.p),'distortion_model':m.distortion_model}
                    elif topic==CAMERA and start<=header<end:
                        if header<min(next_sample,next_video):continue
                        frame=decode_video_frame(m,entry.schema.name,entry.message.log_time)
                        image=Image.frombytes('RGB',(frame.width,frame.height),frame.payload,'raw',{'rgb24':'RGB','bgr24':'BGR'}[frame.input_format])
                        if header>=next_video:
                            video.stdin.write(image.resize((640,360)).tobytes());video_index.append({'frame':len(video_index),'header_time_ns':str(header)})
                            while next_video<=header:next_video+=100_000_000
                        if header>=next_sample:
                            idx=len(camera);relative=f'{ep["id"]}/images/{idx:04d}.png';image.save(output/relative)
                            camera.append({**metadata(entry),'image':relative,'index':idx,'width':frame.width,'height':frame.height,
                                'elapsed_s':(header-start)/1e9,'requested_sample_ns':str(next_sample)})
                            while next_sample<=header:next_sample+=500_000_000
                    elif topic==LIDAR:
                        times=point_times(m);xyz=raw_xyz(m)
                        if len(xyz)!=len(times):raise ValueError('point/time shape mismatch')
                        first,last=int(times.min()),int(times.max())
                        scans.append({**metadata(entry),'xyz':xyz,'point_time_ns':times,'start_ns':first,'end_ns':last,
                                      'mid_ns':(first+last)//2})
                    elif topic==IMU:
                        imus.append({**metadata(entry),'angular_velocity':[m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z],
                            'linear_acceleration':[m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z]})
                    elif topic==GPS:
                        gps.append({**metadata(entry),'velocity':[m.twist.twist.linear.x,m.twist.twist.linear.y,m.twist.twist.linear.z],
                                    'covariance':list(m.twist.covariance)})
            finally:
                video.stdin.close()
                if video.wait():raise RuntimeError('video encoding failed')
            if not camera or not scans or not info:raise ValueError('missing camera/LiDAR/calibration stream')
            for frame in camera:
                t=int(frame['header_time_ns']);scan=min(scans,key=lambda v:abs(v['mid_ns']-t));imu=min(imus,key=lambda v:abs(int(v['header_time_ns'])-t))
                delta=(scan['mid_ns']-t)/1e9
                relative=f'{ep["id"]}/lidar/{frame["index"]:04d}.npz'
                valid=np.isfinite(scan['xyz']).all(1)&(np.linalg.norm(scan['xyz'],axis=1)>1e-6)
                np.savez_compressed(output/relative,xyz_sensor_m=scan['xyz'],point_timestamp_ns=scan['point_time_ns'],valid_point_mask=valid)
                cloud_meta={k:v for k,v in scan.items() if k not in ['xyz','point_time_ns','start_ns','end_ns','mid_ns']}
                cloud_meta.update(path=relative,point_count=len(scan['xyz']),valid_point_count=int(valid.sum()),scan_start_ns=str(scan['start_ns']),scan_end_ns=str(scan['end_ns']),
                    acquisition_midpoint_ns=str(scan['mid_ns']),camera_delta_ms=delta*1000,
                    camera_inside_scan=scan['start_ns']<=t<=scan['end_ns'],association_valid=abs(delta)<=.06,
                    association_method='nearest per-point acquisition midpoint; no deskew; not hardware synchronization')
                g=min(gps,key=lambda v:abs(int(v['log_time_ns'])-int(frame['log_time_ns']))) if gps else None
                if g:
                    g=dict(g);g['header_minus_log_s']=(int(g['header_time_ns'])-int(g['log_time_ns']))/1e9
                    g['usable_for_temporal_training']=abs(g['header_minus_log_s'])<1
                    g['use']='diagnostic only; not used as recorded command or teacher label'
                rows.append({'id':f'{ep["id"]}-{frame["index"]:04d}','episode':ep['id'],'source_file':path.name,
                    'camera':frame,'lidar':cloud_meta,'imu':imu,'gps_velocity':g,'recorded_command':None,
                    'recorded_command_status':'absent_in_source_topic_inventory'})
            write_json(directory/'camera_info.json',info);write_json(directory/'video_index.json',video_index)
            clips.append({**ep,'start_ns':str(start),'end_ns':str(end),'sample_count':len(camera),'video_frames':len(video_index),
                'video':'context.mp4','sampling_hz':2,'video_nominal_hz':10,'video_timestamps':'video_index.json'})
            tiles=[]
            for frame in camera:
                tile=Image.new('RGB',(480,292),'#15232b');im=Image.open(output/frame['image']).resize((480,270));tile.paste(im,(0,0))
                ImageDraw.Draw(tile).text((8,274),f'{ep["id"]} {frame["index"]:04d} +{frame["elapsed_s"]:.2f}s',fill='white');tiles.append(tile)
            for page in range((len(tiles)+11)//12):
                page_tiles=tiles[page*12:(page+1)*12];sheet=Image.new('RGB',(1440,292*((len(page_tiles)+2)//3)),'#15232b')
                for j,tile in enumerate(page_tiles):sheet.paste(tile,((j%3)*480,(j//3)*292))
                sheet.save(directory/f'contact-{page+1}.jpg',quality=91)
            print(json.dumps({'episode':ep['id'],'samples':len(camera),'scans_read':len(scans),'gps_samples':len(gps)},ensure_ascii=False),flush=True)
        del scans
    with (output/'raw_samples.jsonl').open('w') as f:
        for row in rows:f.write(json.dumps(row,ensure_ascii=False,allow_nan=False)+'\n')
    write_json(output/'sources.json',inventories);write_json(output/'episodes.json',clips)
    base,camera=a2_livox_front_transforms('livox_frame','base_link','camera_optical_frame')
    write_json(output/'calibration_assumptions.json',{'profile':'livox-a2-front','source':'tools/lidar_common.py; measured profile documented there',
        'base_from_lidar':base.tolist(),'camera_from_base':camera.tolist(),'verified_for_these_recordings':False,
        'frame_convention':'base x forward, y left, z up; positive yaw left',
        'limitations':['No recorded TF in these files. Mount equivalence to profile is assumed, not established.',
                       'Camera K/D are recorded. LiDAR projection and metric teacher labels depend on the assumed extrinsics.',
                       'No scan deskew or ego-motion compensation. No 360-degree input in this dataset.']})

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--source',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();extract(a.source,a.output)

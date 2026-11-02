#!/usr/bin/env python3
"""Resumable front-camera dataset extraction. Reads MCAP, never publishes ROS.

Source inventory is audited first (including full hashes of duplicate candidates).
Every selected image is lossless, every cloud keeps aligned fields and point time.
"""
import argparse
import bisect
from concurrent.futures import ThreadPoolExecutor
import hashlib
import io
import json
from pathlib import Path
import re
import time

import cv2
import numpy as np
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory

from build_road_sample_dataset import raw_xyz, point_times, metadata
from lidar_common import stamp_ns, a2_livox_front_transforms
from rosbag_mcap_to_mp4 import decode_video_frame

VERSION='connectum.front.raw.v1'


def dump(path,value):
    temporary=path.with_name(path.name+'.tmp')
    temporary.write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    temporary.replace(path)


def line(value):return json.dumps(value,ensure_ascii=False,allow_nan=False)+'\n'


def source_ids(item):
    match=re.search(r'a2_record_(?:default|mapping)_(\d{8})_(\d{6})_(\d+)(?:-\d+)?\.mcap$',Path(item['path']).name)
    if not match:raise ValueError('Unknown recording filename: '+item['path'])
    date,clock,part=match.groups()
    return f'{date}_{clock}',f'{date}_{clock}_p{int(part):03d}'


def nearest(values,times,stamp):
    if not values:return None
    at=bisect.bisect_left(times,stamp)
    choices=[i for i in (at-1,at) if 0<=i<len(values)]
    return values[min(choices,key=lambda i:abs(times[i]-stamp))]


def field_arrays(message):
    formats={1:'i1',2:'u1',3:'i2',4:'u2',5:'i4',6:'u4',7:'f4',8:'f8'}
    result={}
    for field in message.fields:
        if field.name in ('x','y','z','timestamp') or field.count!=1:continue
        dtype=np.dtype({'names':['v'],'formats':[('>' if message.is_bigendian else '<')+formats[field.datatype]],
                        'offsets':[field.offset],'itemsize':message.point_step})
        result[field.name]=np.ndarray((message.height,message.width),dtype=dtype,buffer=bytes(message.data),
            strides=(message.row_step,message.point_step))['v'].reshape(-1).copy()
    return result


def raw_point_times_with_issue(message):
    """Preserve a mixed-time scan without treating its midpoint as synchronized."""
    field=next((f for f in message.fields if f.name=='timestamp'),None)
    if field is None or field.datatype!=8 or field.count!=1:
        raise ValueError('Recorded scalar float64 point timestamp required')
    dtype=np.dtype({'names':['t'],'formats':['>f8' if message.is_bigendian else '<f8'],
        'offsets':[field.offset],'itemsize':message.point_step})
    values=np.ndarray((message.height,message.width),dtype=dtype,buffer=bytes(message.data),
        strides=(message.row_step,message.point_step))['t'].reshape(-1).copy()
    finite=np.isfinite(values)
    issue=None
    if not len(values) or not finite.all():issue='nonfinite_or_empty_point_timestamps'
    elif abs(float(values.min())-stamp_ns(message.header))>2e8:issue='point_timestamp_header_mismatch'
    elif np.ptp(values)>2e8:issue='mixed_time_scan_over_200ms'
    return values,issue


def write_image(path,frame):
    a=np.frombuffer(frame.payload,np.uint8).reshape(frame.height,frame.width,3)
    rgb=a if frame.input_format=='rgb24' else a[:,:,::-1]
    ok,encoded=cv2.imencode('.png',rgb[:,:,::-1],[cv2.IMWRITE_PNG_COMPRESSION,1])
    if not ok:raise RuntimeError('PNG encoding failed')
    data=encoded.tobytes();path.write_bytes(data)
    return {'sha256':hashlib.sha256(data).hexdigest(),'decoded_rgb_sha256':hashlib.sha256(rgb.tobytes()).hexdigest(),
            'bytes':len(data),'encoding':'lossless_png','width':frame.width,'height':frame.height}


def write_cloud(path,scan):
    xyz=scan['xyz'];valid=np.isfinite(xyz).all(1)&(np.linalg.norm(xyz,axis=1)>1e-6)
    data=io.BytesIO()
    np.savez_compressed(data,xyz_sensor_m=xyz,point_timestamp_ns=scan['times'],valid_point_mask=valid,**scan['fields'])
    payload=data.getvalue();path.write_bytes(payload)
    return {'sha256':hashlib.sha256(payload).hexdigest(),'bytes':len(payload),'point_count':len(xyz),
            'valid_point_count':int(valid.sum()),'additional_fields':list(scan['fields'])}


def imu_entry(entry):
    m=entry.decoded_message
    return {**metadata(entry),'angular_velocity':[m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z],
            'linear_acceleration':[m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z],
            'orientation_xyzw':[m.orientation.x,m.orientation.y,m.orientation.z,m.orientation.w]}


def tf_entry(entry):
    return {'topic':entry.channel.topic,'log_time_ns':str(entry.message.log_time),
            'message_sha256':hashlib.sha256(entry.message.data).hexdigest(),'transforms':[
        {'header_time_ns':str(stamp_ns(t.header)),'parent':t.header.frame_id,'child':t.child_frame_id,
         'translation':[t.transform.translation.x,t.transform.translation.y,t.transform.translation.z],
         'rotation_xyzw':[t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w]}
        for t in entry.decoded_message.transforms]}


def extract_file(item,root,hz,last_bucket):
    session,source_id=source_ids(item);directory=root/'sequences'/source_id;directory.mkdir(parents=True,exist_ok=True)
    for sub in ['images','lidar','previews']:(directory/sub).mkdir(exist_ok=True)
    complete=directory/'completed.json'
    if complete.exists():
        status=json.loads(complete.read_text())
        if status['version']!=VERSION or status['hz']!=hz or status['source_bytes']!=item['bytes'] or status['source_mtime_ns']!=item['mtime_ns']:
            raise ValueError('Resume contract mismatch: '+source_id)
        assert hashlib.sha256((directory/'samples.jsonl').read_bytes()).hexdigest()==status['index_sha256']
        print(line({'event':'resume','sequence':source_id,'samples':status['samples']}),end='',flush=True)
        return status['last_bucket']
    camera_topics=[t for t,v in item['topics'].items() if v['type']=='sensor_msgs/msg/Image' and 'front_camera' in t]
    if len(camera_topics)!=1:raise ValueError('Need exactly one front camera topic: '+source_id)
    camera_topic=camera_topics[0]
    info_topics=[t for t,v in item['topics'].items() if v['type']=='sensor_msgs/msg/CameraInfo']
    lidar_topics=[t for t,v in item['topics'].items() if v['type']=='sensor_msgs/msg/PointCloud2']
    if len(lidar_topics)>1:raise ValueError('Select LiDAR explicitly for multi-lidar recording')
    has_lidar=bool(lidar_topics)
    period=round(1e9/hz);selected=[];scans=[];infos=[];imus=[];gps=[]
    start=time.monotonic();camera_count=0;clock_reversals=0;last_camera=0;schema=None;quarantined=[]
    topics=camera_topics+info_topics+lidar_topics+[t for t in ['/livox/imu','/navheading','/ublox_gps_node/fix_velocity','/tf','/tf_static'] if t in item['topics']]
    futures=[];source=Path(item['path'])
    with (directory/'camera_timeline.jsonl').open('w') as timeline,(directory/'telemetry.jsonl').open('w') as telemetry,source.open('rb') as stream,ThreadPoolExecutor(max_workers=3) as pool:
        reader=make_reader(stream,decoder_factories=[DecoderFactory()])
        for entry in reader.iter_decoded_messages(topics=topics):
            topic=entry.channel.topic;m=entry.decoded_message
            if topic==camera_topic:
                t=stamp_ns(m.header);camera_count+=1
                timeline.write(line({'header_time_ns':str(t),'log_time_ns':str(entry.message.log_time),'frame_id':m.header.frame_id,'source_sequence':entry.message.sequence}))
                if t<=last_camera:clock_reversals+=1
                last_camera=t;bucket=t//period
                if bucket<=last_bucket:continue
                last_bucket=bucket
                index=len(selected);name=f'{index:06d}'
                frame=decode_video_frame(m,entry.schema.name,entry.message.log_time)
                relative=(directory/'images'/f'{name}.png').relative_to(root).as_posix()
                frame_meta={**metadata(entry),'image':relative,'index':index,'sampling_bucket':str(bucket),
                    'header_minus_log_ms':(t-entry.message.log_time)/1e6,'source_encoding':m.encoding,'source_sequence':entry.message.sequence}
                future=pool.submit(write_image,root/relative,frame);futures.append((frame_meta,future))
                if len(futures)>10:
                    meta,job=futures.pop(0);meta.update(job.result())
                selected.append(frame_meta)
            elif topic in lidar_topics:
                times,issue=raw_point_times_with_issue(m);xyz=raw_xyz(m)
                if len(xyz)!=len(times):raise ValueError('LiDAR point/time misalignment')
                if issue:
                    quarantine=directory/'quarantine';quarantine.mkdir(exist_ok=True)
                    dest=quarantine/f'lidar-{len(quarantined):06d}.npz'
                    stats=write_cloud(dest,{'xyz':xyz,'times':times,'fields':field_arrays(m)})
                    finite=times[np.isfinite(times)]
                    quarantined.append({**metadata(entry),**stats,'path':dest.relative_to(root).as_posix(),
                        'reason':issue,'used_for_association':False,'finite_timestamp_points':len(finite),
                        'point_start_ns':str(int(finite.min())) if len(finite) else None,
                        'point_end_ns':str(int(finite.max())) if len(finite) else None,
                        'point_span_s':float(np.ptp(finite)/1e9) if len(finite) else None,
                        'note':'Exact raw scan preserved separately; excluded from camera association and pseudo commands.'})
                    print(line({'event':'quarantined_lidar_scan','sequence':source_id,'reason':issue,
                        'point_span_s':quarantined[-1]['point_span_s']}),end='',flush=True)
                    continue
                first,last=int(times.min()),int(times.max())
                scans.append({**metadata(entry),'xyz':xyz,'times':times,'fields':field_arrays(m),
                    'scan_start_ns':str(first),'scan_end_ns':str(last),'midpoint_ns':str((first+last)//2)})
                if schema is None:
                    schema={'point_step':m.point_step,'fields':[{'name':f.name,'offset':f.offset,'datatype':f.datatype,'count':f.count} for f in m.fields],
                            'is_bigendian':m.is_bigendian,'point_timestamp_unit':'absolute Unix nanoseconds stored as float64'}
            elif topic in info_topics:
                signature=[m.width,m.height,list(m.k),list(m.d),list(m.r),list(m.p),m.distortion_model]
                if not infos or signature!=infos[-1]['signature']:
                    infos.append({**metadata(entry),'width':m.width,'height':m.height,'K':list(m.k),'D':list(m.d),'R':list(m.r),'P':list(m.p),
                        'distortion_model':m.distortion_model,'signature':signature})
            elif topic in ['/livox/imu','/navheading']:
                record=imu_entry(entry);telemetry.write(line(record))
                if topic=='/livox/imu':imus.append(record)
            elif topic=='/ublox_gps_node/fix_velocity':
                record={**metadata(entry),'velocity':[m.twist.twist.linear.x,m.twist.twist.linear.y,m.twist.twist.linear.z],
                    'covariance':list(m.twist.covariance),'header_minus_log_s':(stamp_ns(m.header)-entry.message.log_time)/1e9,
                    'used_for_commands':False}
                record['usable_timebase']=abs(record['header_minus_log_s'])<1
                telemetry.write(line(record));gps.append(record)
            elif topic in ['/tf','/tf_static']:
                telemetry.write(line(tf_entry(entry)))
        for meta,job in futures:meta.update(job.result())
    if not selected or not infos:raise ValueError('Missing selected image or camera calibration')
    scans.sort(key=lambda s:int(s['midpoint_ns']));scan_times=[int(s['midpoint_ns']) for s in scans]
    imus.sort(key=lambda v:int(v['header_time_ns']));imu_times=[int(v['header_time_ns']) for v in imus]
    gps.sort(key=lambda v:int(v['log_time_ns']));gps_times=[int(v['log_time_ns']) for v in gps]
    for info in infos:info.pop('signature')
    dump(directory/'camera_info.json',infos);dump(directory/'lidar_schema.json',schema)
    dump(directory/'quarantined_scans.json',quarantined)
    rows=[];cached_clouds={}
    for camera in selected:
        t=int(camera['header_time_ns']);scan=nearest(scans,scan_times,t)
        cloud=None
        if scan is not None:
            key=scan['message_sha256']
            if key not in cached_clouds:
                rel=(directory/'lidar'/f"{camera['index']:06d}.npz").relative_to(root).as_posix()
                cached_clouds[key]={'path':rel,**write_cloud(root/rel,scan)}
            delta=(int(scan['midpoint_ns'])-t)/1e6
            cloud={**{k:v for k,v in scan.items() if k not in ['xyz','times','fields']},**cached_clouds[key],
                   'camera_delta_ms':delta,'association_valid':abs(delta)<=60,
                   'camera_inside_scan':int(scan['scan_start_ns'])<=t<=int(scan['scan_end_ns']),
                   'association_method':'nearest per-point acquisition midpoint within this source file; no deskew'}
        info_index=min(range(len(infos)),key=lambda i:abs(int(infos[i]['header_time_ns'])-t))
        imu=nearest(imus,imu_times,t);g=nearest(gps,gps_times,int(camera['log_time_ns']))
        if imu:imu={**imu,'camera_delta_ms':(int(imu['header_time_ns'])-t)/1e6}
        if g:g={**g,'association_method':'arrival-time diagnostic only; not used for training alignment'}
        rows.append({'schema_version':VERSION,'id':f"{source_id}_{camera['index']:06d}",'session':session,'sequence':source_id,
            'source_relative_path':item['relative_path'],'camera':camera,'camera_info':{'path':(directory/'camera_info.json').relative_to(root).as_posix(),'index':info_index},
            'lidar':cloud,'lidar_status':'present' if cloud else 'topic_absent_in_source',
            'imu':imu,'gps_velocity':g,'recorded_command':None,'recorded_command_status':'no_control_topic_in_source',
            'observed_motion':None,'camera_views':['front'],'is_360_camera':False})
    temporary=directory/'samples.jsonl.tmp'
    with temporary.open('w') as f:
        for row in rows:f.write(line(row))
    temporary.replace(directory/'samples.jsonl')
    if source.stat().st_size!=item['bytes'] or str(source.stat().st_mtime_ns)!=item['mtime_ns']:raise ValueError('Source changed while reading')
    # Six representative raw frames per source for quick visual auditing.
    chosen=sorted(set(np.linspace(0,len(rows)-1,min(6,len(rows))).round().astype(int)))
    tiles=[]
    for idx in chosen:
        r=rows[idx];im=cv2.imread(str(root/r['camera']['image']));tile=np.full((204,320,3),24,np.uint8)
        tile[:180]=cv2.resize(im,(320,180));cv2.putText(tile,f"{idx:06d}  {r['camera']['header_time_ns']}",(5,196),cv2.FONT_HERSHEY_SIMPLEX,.32,(230,230,230),1);tiles.append(tile)
    sheet=np.full((204*((len(tiles)+2)//3),960,3),24,np.uint8)
    for j,tile in enumerate(tiles):sheet[(j//3)*204:(j//3+1)*204,(j%3)*320:(j%3+1)*320]=tile
    cv2.imwrite(str(directory/'previews'/'contact.jpg'),sheet)
    status={'version':VERSION,'hz':hz,'source_bytes':item['bytes'],'source_mtime_ns':item['mtime_ns'],
        'session':session,'sequence':source_id,'source_relative_path':item['relative_path'],'samples':len(rows),'native_camera_messages':camera_count,
        'lidar_scans_read':len(scans)+len(quarantined),'saved_lidar_scans':len(cached_clouds),
        'quarantined_lidar_scans':len(quarantined),'has_lidar':has_lidar,'camera_clock_reversals':clock_reversals,
        'invalid_lidar_associations':sum(r['lidar'] is not None and not r['lidar']['association_valid'] for r in rows),
        'last_bucket':last_bucket,'index_sha256':hashlib.sha256((directory/'samples.jsonl').read_bytes()).hexdigest(),
        'seconds':round(time.monotonic()-start,2)}
    dump(complete,status);print(line({'event':'extracted',**status}),end='',flush=True)
    return last_bucket


def run(root,hz):
    items=json.loads((root/'source_inventory.json').read_text())
    selected=sorted([i for i in items if 'duplicate_of' not in i],key=lambda i:int(i['start_ns']))
    for item in selected:
        if any(any(k in t.lower() for k in ['cmd_vel','sport/request','command','control']) for t in item['topics']):
            raise ValueError('Source includes commands: add a decoder rather than label them absent')
    base,cam=a2_livox_front_transforms('livox_frame','base_link','camera_optical_frame')
    dump(root/'calibration_assumptions.json',{'profile':'livox-a2-front','base_from_lidar':base.tolist(),'camera_from_base':cam.tolist(),
        'verified_for_these_recordings':False,'source':'tools/lidar_common.py','camera_intrinsics':'per-source recorded CameraInfo',
        'limits':['Lidar-to-base extrinsics assumed from local profile, not proven by these bags.',
                  'Some camera-only sessions have camera TF; it does not establish the LiDAR mount in other sessions.',
                  'No scan deskew; missing sensors remain null; native point timestamps preserved.']})
    dump(root/'extraction_config.json',{'schema':VERSION,'target_hz':hz,'image_format':'lossless PNG, native resolution',
        'sampling':'first frame in each epoch-aligned 1/hz acquisition-time bin, per recording session',
        'sources_considered':len(items),'unique_sources':len(selected),'source_root':'/Volumes/MY_HDD/20261021_dangjin',
        'generated_at_timezone':'Asia/Seoul','command_topics_present':False,'camera_views':['front']})
    last={}
    for i,item in enumerate(selected):
        session,source_id=source_ids(item)
        print(line({'event':'start','file_number':i+1,'files':len(selected),'sequence':source_id}),end='',flush=True)
        last[session]=extract_file(item,root,hz,last.get(session,-1))
    all_rows=[];statuses=[]
    for item in selected:
        _,source_id=source_ids(item);directory=root/'sequences'/source_id
        all_rows.extend(json.loads(s) for s in (directory/'samples.jsonl').read_text().splitlines())
        statuses.append(json.loads((directory/'completed.json').read_text()))
    with (root/'samples.jsonl.tmp').open('w') as f:
        for row in all_rows:f.write(line(row))
    (root/'samples.jsonl.tmp').replace(root/'samples.jsonl')
    dump(root/'extraction_complete.json',{'status':'complete','samples':len(all_rows),'sources':len(statuses),'source_status':statuses,
        'with_lidar':sum(r['lidar'] is not None for r in all_rows),'camera_only':sum(r['lidar'] is None for r in all_rows)})
    print(line({'event':'all_extracted','samples':len(all_rows)}),end='',flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True);p.add_argument('--hz',type=float,default=5.)
    a=p.parse_args()
    if not 0<a.hz<=20:raise ValueError('hz must be in (0,20]')
    run(a.output,a.hz)

#!/usr/bin/env python3
"""Read selected September MCAP files into bounded, provenance-bearing samples.

GPS samples retain their measured coordinates and uncertainty. No assumption
that their coordinate frame matches the hand-modelled site is made here.
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory

from lidar_common import cloud_xyz
from rosbag_mcap_to_mp4 import decode_video_frame

CAMERA = '/a2/front_camera/image_raw'
GPS = '/ublox_gps_node/fix'


def sample(reader, topic, target, window=2_000_000_000):
    return next(reader.iter_decoded_messages(topics=[topic], start_time=target, end_time=target+window), None)


def inspect_files(source):
    files=[]
    for path in source.glob('a2_record_default_20260915_153503_*-*.mcap'):
        with path.open('rb') as stream:
            summary=make_reader(stream).get_summary()
            if not summary or not summary.statistics:raise ValueError(f'Missing indexed summary: {path.name}')
            stats=summary.statistics
            topics={}
            for cid,channel in summary.channels.items():
                count=stats.channel_message_counts.get(cid,0)
                if count:
                    entry=topics.setdefault(channel.topic,{'count':0,'type':summary.schemas[channel.schema_id].name})
                    entry['count']+=count
            files.append({'file':path.name,'bytes':path.stat().st_size,
                          'start_ns':str(stats.message_start_time),'end_ns':str(stats.message_end_time),
                          'duration_s':(stats.message_end_time-stats.message_start_time)/1e9,'topics':topics})
    if not files:raise ValueError('No matching September split MCAP files')
    return sorted(files,key=lambda f:int(f['start_ns']))


def extract(source, output, inventory):
    output.mkdir(parents=True, exist_ok=True)
    files = json.loads(inventory.read_text()) if inventory else inspect_files(source)
    origin_ns = min(int(f['start_ns']) for f in files)
    frames, gps, clouds, tiles = [], [], [], []
    for index, file in enumerate(files):
        with (source/file['file']).open('rb') as stream:
            reader = make_reader(stream, decoder_factories=[DecoderFactory()])
            start, end = int(file['start_ns']), int(file['end_ns'])
            for j, fraction in enumerate((.1,.5,.9)):
                target = start+int((end-start)*fraction)
                entry = sample(reader, CAMERA, target)
                if entry:
                    frame = decode_video_frame(entry.decoded_message, entry.schema.name, entry.message.log_time)
                    image = Image.frombytes('RGB', (frame.width,frame.height), frame.payload,
                                            'raw', {'rgb24':'RGB','bgr24':'BGR'}[frame.input_format])
                    # The gallery keeps the native image; contact sheets are previews.
                    name = f'camera-{index:02d}-{j:02d}.jpg';image.save(output/name, quality=86)
                    frames.append({'file':file['file'],'image':name,'timestamp_ns':str(entry.message.log_time),
                                   'elapsed_s':round((entry.message.log_time-origin_ns)/1e9,3),
                                   'frame_id':entry.decoded_message.header.frame_id,'width':frame.width,'height':frame.height})
                    tile=Image.new('RGB',(384,238),'#152b30');tile.paste(image.resize((384,216)),(0,0))
                    ImageDraw.Draw(tile).text((6,220),f"{index:02d}.{j}  part {file['file'].split('_')[-1]}  {(entry.message.log_time-origin_ns)/1e9:.1f}s",fill='white');tiles.append(tile)
                entry = sample(reader,GPS,target)
                if entry:
                    msg=entry.decoded_message
                    gps.append({'file':file['file'],'timestamp_ns':str(entry.message.log_time),
                                'elapsed_s':round((entry.message.log_time-origin_ns)/1e9,3),
                                'latitude':msg.latitude,'longitude':msg.longitude,'altitude':msg.altitude,
                                'status':msg.status.status,'service':msg.status.service,
                                'covariance_type':msg.position_covariance_type,'covariance':list(msg.position_covariance)})
            entry = sample(reader,'/livox/lidar',(start+end)//2,10_000_000_000)
            if entry:
                xyz=cloud_xyz(entry.decoded_message);n=len(xyz)
                xyz=xyz[np.linspace(0,n-1,min(n,6000),dtype=int)]
                name=f'lidar-{index:02d}.json';(output/name).write_text(json.dumps(np.round(xyz,3).tolist(),separators=(',',':')))
                clouds.append({'file':file['file'],'points':name,'timestamp_ns':str(entry.message.log_time),
                               'elapsed_s':round((entry.message.log_time-origin_ns)/1e9,3),
                               'original_points':n,'displayed_points':len(xyz),'frame_id':entry.decoded_message.header.frame_id})
            entry=sample(reader,'/a2/front_camera/camera_info',start)
            if entry:
                msg=entry.decoded_message
                file['camera_info']={'width':msg.width,'height':msg.height,'k':list(msg.k),'d':list(msg.d),'distortion_model':msg.distortion_model}
        print(f"{index+1}/{len(files)} {file['file']}",flush=True)
    valid=[p for p in gps if p['status']>=0 and all(math.isfinite(p[k]) for k in ('latitude','longitude','altitude')) and abs(p['latitude'])>1]
    origin=None
    if valid:
        origin={'latitude':valid[0]['latitude'],'longitude':valid[0]['longitude'],'altitude':valid[0]['altitude']}
        for p in valid:
            p['enu_m']=[6378137*math.radians(p['longitude']-origin['longitude'])*math.cos(math.radians(origin['latitude'])),
                        6378137*math.radians(p['latitude']-origin['latitude']),p['altitude']-origin['altitude']]
    manifest={'schema_version':1,'recorded_date':'2026-09-15','files':files,'frames':frames,'clouds':clouds,
              'gps_samples':gps,'gps_origin':origin,'registration':'none','sampling':'3 camera/GPS samples and 1 raw LiDAR scan per supplied file',
              'coverage_s':sum(f['duration_s'] for f in files),
              'timeline_span_s':(max(int(f['end_ns']) for f in files)-origin_ns)/1e9,
              'limitations':['Only supplied split files; temporal gaps remain.','GPS is not aligned to the site plan.','Raw point clouds are independent sensor frames.']}
    (output/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
    for page in range((len(tiles)+23)//24):
        subset=tiles[page*24:(page+1)*24];sheet=Image.new('RGB',(1152,238*((len(subset)+2)//3)),'#152b30')
        for i,tile in enumerate(subset):sheet.paste(tile,((i%3)*384,(i//3)*238))
        sheet.save(output/f'contact-{page+1}.jpg',quality=91)
    print(f"Finished: {len(frames)} images, {len(clouds)} scans, {len(valid)}/{len(gps)} valid GPS samples",flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--input',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--inventory',type=Path)
    a=p.parse_args();extract(a.input,a.output,a.inventory)

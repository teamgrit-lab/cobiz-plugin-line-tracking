#!/usr/bin/env python3
"""Local-only 3D simulator + real pretrained Flyvis planner. Never imports ROS.

The browser sends synthetic RGB camera pixels. Road perception estimates its
own masks; runtime requests containing ground-truth masks are rejected. This
service reuses the production optical-flow circuit and surface planner, then
tracks the visible path with a simulation-only pure-pursuit controller.
Simulated time pauses while inference is pending.
"""
from __future__ import annotations
import argparse
from dataclasses import asdict
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import threading
import time
import base64

import cv2
import numpy as np
from connectome_server import PlannerService, MAX_BODY, decode_image
from connectome_surface import SurfaceArea, PlannerConfig
from local_path import LocalPathConfig
from swin_l_drive_control import DriveDecision

ROOT = Path(__file__).resolve().parents[1]
def encode_png(image):return base64.b64encode(cv2.imencode('.png',image)[1]).decode()


BUNDLE_SHA256 = '97f853183f79d2e987532ca2716ce3c33c2f41f875af05396e0cd8b14a7bc3c2'


def rectangle_distance(x, y, r):
    return math.hypot(max(r['x']-x, 0, x-r['x']-r['length']),
                      max(r['y']-y, 0, y-r['y']-r['width']))


def validate_pose(pose):
    if not isinstance(pose, dict) or any(type(pose.get(k)) not in (int,float) or not math.isfinite(pose[k]) for k in ('x','y','heading')):
        raise ValueError('finite simulation pose required')
    if abs(pose['x'])>1000 or abs(pose['y'])>1000:
        raise ValueError('pose outside simulator bounds')


def to_site(points, pose):
    c,s=math.cos(pose['heading']),math.sin(pose['heading'])
    p=np.asarray(points,float)
    return np.column_stack((pose['x']+c*p[:,0]-s*p[:,1],pose['y']+s*p[:,0]+c*p[:,1]))


def forbidden_path(points, zones, radius):
    p=np.asarray(points,float)
    if p.ndim!=2 or p.shape[1]!=2 or not len(p) or not np.isfinite(p).all():
        return True
    for i in range(max(1,len(p)-1)):
        a,b=p[i],p[min(i+1,len(p)-1)]
        for xy in np.linspace(a,b,max(2,math.ceil(float(np.linalg.norm(b-a))/.1)+1)):
            if any(rectangle_distance(*xy,z)<=radius for z in zones):return True
    return False


class SimulationService:
    def __init__(self, encoder, site, perception, world):
        self.service=PlannerService(encoder);self.site=site;self.lock=threading.Lock();self.perception=perception
        self.world=world;self.trace=[];self.session=None;self.started=time.monotonic();self.errors=0
        self.patrol=None;self.mission_id=None;self.mission=None

    def step(self, request):
        from dangjin_patrol import Patrol,track_camera_path
        pose=request.get('pose');validate_pose(pose)
        if request.get('requested_modes')!=[1]:raise ValueError('simulation requires explicit road mode')
        if any(k in request for k in ('selected_png','hazard_png','corridor_xy','target_xy')):
            raise ValueError('runtime accepts RGB only; client masks and ground-truth paths are forbidden')
        yaw=request.get('camera_yaw',0.)
        if type(yaw) not in (int,float) or not math.isfinite(yaw) or abs(yaw)>1.3:raise ValueError('bounded virtual-camera yaw required')
        camera_pose={**pose,'heading':pose['heading']+yaw}
        speed=request.get('speed_mps',.8)
        if type(speed) not in (int,float) or not .1<=speed<=1.:raise ValueError('sim speed must be 0.1 to 1.0 m/s')
        mission_id=request.get('mission_id',request['stream_id'])
        if mission_id!=self.mission_id:
            self.mission_id=mission_id;self.trace=[];self.started=time.monotonic()
            self.patrol=Patrol(self.world['perimeter_route']) if request.get('scenario')=='patrol' else None
        self.mission=self.patrol.update(pose) if self.patrol else None
        frame=decode_image(request['image_png'],cv2.IMREAD_COLOR)
        surround_preview=None
        if hasattr(self.perception,'infer_views'):
            views=request.get('views_png')
            if not isinstance(views,list) or len(views)!=4 or views[0]!=request['image_png']:
                raise ValueError('four ordered camera views required; view 0 must match tracking RGB')
            selected,hazard,metrics,surround_preview=self.perception.infer_views([decode_image(v,cv2.IMREAD_COLOR) for v in views])
        else:selected,hazard,metrics=self.perception.infer(frame)
        local_values=dict(request['local_config'])
        for key in ('roi_polygon','road_roi_polygon','roi_exclusion_polygon','road_roi_exclusion_polygon'):
            if local_values.get(key) is not None:local_values[key]=tuple(local_values[key])
        local=LocalPathConfig(**local_values);config=PlannerConfig(**request['planner_config'])
        planner=dict(request,selected_png=encode_png(selected),hazard_png=encode_png(hazard))
        if self.mission:
            dx,dy=np.array(self.mission['target_site'])-[pose['x'],pose['y']]
            c,s=math.cos(camera_pose['heading']),math.sin(camera_pose['heading'])
            planner['target_xy']=[float(c*dx+s*dy),float(-s*dx+c*dy)]
        reply=self.service.process(planner)
        path=reply['paths']['1'];reason=None
        zones=self.site.get('no_go_zones',[]);radius=self.site['simulation']['robot_radius_m']
        if forbidden_path([[pose['x'],pose['y']]],zones,radius):reason='no_go_zone'
        if path:
            points=np.asarray(path['points_xy'],np.float32)
            if not SurfaceArea(selected,hazard,1,local,config).accepts(points):reason='fresh_mask_violation'
            if forbidden_path(to_site(points,camera_pose),zones,radius):reason='no_go_zone'
            command=track_camera_path(points,yaw,speed)
        else:command=DriveDecision.stop('path_unavailable')
        if reason:command=DriveDecision.stop(reason)
        if self.mission and self.mission['complete']:command=DriveDecision.stop('lap_complete')
        self.trace.append({'stamp_sec':request['stamp_sec'],'pose':pose,'camera_yaw':yaw,'command':asdict(command),
                           'confidence':path['confidence'] if path else 0,'path_end':path['points_xy'][-1] if path else None,
                           'flow_ms':reply['timing']['flow_ms'],'surface_ms':reply['timing']['surface_ms'],
                           'perception':metrics,'mission':self.mission})
        if len(self.trace)>20000:self.trace=self.trace[-20000:]
        preview=np.full((*selected.shape,3),(50,48,27),np.uint8);preview[selected!=0]=(140,167,94)
        folder=ROOT/'.cache/dangjin-perception';folder.mkdir(exist_ok=True)
        if len(self.trace)==1 or len(self.trace)%100==0 or command.vx==0:
            cv2.imwrite(str(folder/'latest-rgb.png'),frame);cv2.imwrite(str(folder/'latest-road.png'),selected*255)
        reply.update(command=asdict(command),simulation_only=True,perception=self.perception.details,
                     perception_metrics=metrics,prediction_png=encode_png(preview),mission=self.mission,
                     control='RGB path pure pursuit',no_go_zone_count=len(zones))
        if surround_preview is not None:reply['surround_prediction_png']=encode_png(surround_preview)
        return reply

    def telemetry(self):
        distance=sum(math.hypot(b['pose']['x']-a['pose']['x'],b['pose']['y']-a['pose']['y']) for a,b in zip(self.trace,self.trace[1:]))
        return {'session':self.mission_id,'layout_revision':self.site.get('layout_revision'),'steps':len(self.trace),'distance_m':distance,'wall_s':time.monotonic()-self.started,
                'errors':self.errors,'latest':self.trace[-1] if self.trace else None,'mission':self.mission,
                'stops':sum(t['command']['vx']==0 for t in self.trace),'model':self.service.encoder.details,
                'perception':self.perception.details,'trace':self.trace}


def make_sim_server(sim, directory, port):
    class Handler(SimpleHTTPRequestHandler):
        def reply(self,status,value):
            data=json.dumps(value,allow_nan=False).encode();self.send_response(status)
            self.send_header('Content-Type','application/json');self.send_header('Cache-Control','no-store')
            self.send_header('Content-Length',str(len(data)));self.end_headers()
            try:self.wfile.write(data)
            except (BrokenPipeError,ConnectionResetError):pass

        def do_GET(self):
            if self.path=='/api/status':
                self.reply(200,{'ready':True,'simulation_only':True,'layout_revision':sim.site.get('layout_revision'),'model':sim.service.encoder.details,'no_go_zones':sim.site.get('no_go_zones',[]),'perception':sim.perception.details});return
            if self.path=='/api/telemetry':self.reply(200,sim.telemetry());return
            super().do_GET()

        def do_POST(self):
            origin=self.headers.get('Origin')
            if origin and origin not in (f'http://127.0.0.1:{port}',f'http://localhost:{port}'):
                self.reply(403,{'error':'same-origin localhost required'});return
            if self.headers.get_content_type()!='application/json':self.reply(415,{'error':'JSON required'});return
            if self.path=='/api/training-frame':
                try:
                    length=int(self.headers.get('Content-Length','0'))
                    if not 0<length<=MAX_BODY:raise ValueError('bounded training frame required')
                    body=json.loads(self.rfile.read(length));index=body['index']
                    surround=body.get('dataset')=='connectome360'
                    if type(index) is not int or not 0<=index<(512 if surround else 256):raise ValueError('invalid sample index')
                    frame=decode_image(body['image_png'],cv2.IMREAD_COLOR);mask=decode_image(body['selected_png'],cv2.IMREAD_GRAYSCALE)
                    if mask.shape!=frame.shape[:2] or not np.isin(mask,[0,1]).all():raise ValueError('aligned binary training labels required')
                    folder=ROOT/('.cache/dangjin-connectome-training' if surround else '.cache/dangjin-training');folder.mkdir(exist_ok=True)
                    cv2.imwrite(str(folder/f'{index:03d}-rgb.png'),frame);cv2.imwrite(str(folder/f'{index:03d}-road.png'),mask)
                    if surround:
                        validate_pose(body.get('pose'))
                        (folder/f'{index:03d}.json').write_text(json.dumps({'index':index,'pose_group':index//4,'sector':index%4,
                            'pose':body['pose'],'layout_revision':sim.site.get('layout_revision'),
                            'label_use':'offline training only'},allow_nan=False)+'\n')
                    self.reply(200,{'saved':index})
                except Exception as error:self.reply(422,{'error':str(error)})
                return
            if self.path!='/api/step':self.reply(404,{'error':'unknown endpoint'});return
            if not sim.lock.acquire(blocking=False):self.reply(409,{'error':'planner busy; no queued control commands'});return
            try:
                length=int(self.headers.get('Content-Length','0'))
                if not 0<length<=MAX_BODY:raise ValueError('bounded request required')
                self.reply(200,sim.step(json.loads(self.rfile.read(length))))
            except Exception as error:
                sim.errors+=1;sim.service.encoder.reset();self.reply(422,{'error':str(error)[:240],'command':asdict(DriveDecision.stop('planner_error'))})
            finally:sim.lock.release()

        def end_headers(self):
            self.send_header('Cache-Control','no-cache');super().end_headers()
        def log_message(self,*args):pass
        def setup(self):super().setup();self.connection.settimeout(10.)
    return ThreadingHTTPServer(('127.0.0.1',port),partial(Handler,directory=str(directory)))


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--port',type=int,default=8766)
    p.add_argument('--bundle',type=Path,default=ROOT/'.cache/flyvis_encoder.pt');p.add_argument('--sha256',default=BUNDLE_SHA256)
    p.add_argument('--device',choices=['cpu','cuda'],default='cpu')
    p.add_argument('--road-backend',choices=['connectome','segformer'],default='connectome')
    p.add_argument('--road-model',type=Path)
    p.add_argument('--road-device',choices=['cpu','mps','cuda'],default='mps')
    p.add_argument('--alias-port',type=int);a=p.parse_args()
    from connectome_flow import FlyvisFlow
    directory=ROOT/'environments/dangjin';site=json.loads((directory/'site.json').read_text())
    cv2.setNumThreads(2);encoder=FlyvisFlow(a.bundle,a.sha256,a.device)
    if a.road_backend=='connectome':
        from connectome_road import ConnectomeRoadPerception
        perception=ConnectomeRoadPerception(encoder,a.road_model or ROOT/'.cache/dangjin-connectome-road',a.road_device)
    else:
        from dangjin_perception import RoadPerception
        perception=RoadPerception(a.road_model or ROOT/'.cache/dangjin-road-model',a.road_device)
    sim=SimulationService(encoder,site,perception,json.loads((directory/'world.json').read_text()))
    if a.alias_port is not None:
        alias=make_sim_server(sim,directory,a.alias_port)
        threading.Thread(target=alias.serve_forever,daemon=True).start()
    print(f'Ready: http://127.0.0.1:{a.port} · verified Flyvis · simulation only',flush=True)
    make_sim_server(sim,directory,a.port).serve_forever()


if __name__=='__main__':main()

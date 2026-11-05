#!/usr/bin/env python3
"""Local-only 3D simulator + real Flyvis perception and learned drive readout.

The default neural mode predicts speed, yaw and stopping from inferred road
masks. It does not run the baseline planner or safety command overrides.
Both front and surround modes use camera-only commands, without an object overlay.
The browser records collisions and departures without correcting motion.
--control baseline retains the original optical-flow/pure-pursuit experiment.
No ROS or physical robot control is imported. Time pauses during inference.
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
from drive_control import DriveDecision
from dangjin_camera import panorama_mask
from surround_camera import load_profile,profile_sha256
from dangjin_sensor_faults import SensorFaults,fault_config

ROOT = Path(__file__).resolve().parents[1]
FCN_MAX_BODY = 12 * 1024 * 1024  # bounded 2048x1024 lossless four-view RGB atlas
def encode_png(image):
    return base64.b64encode(cv2.imencode('.png',image,[cv2.IMWRITE_PNG_COMPRESSION,1])[1]).decode()


def runtime_details(details):
    # Training histories are available once in /api/status and in exports.
    # Do not serialize and send every training epoch with every camera frame.
    return {key:value for key,value in details.items() if key not in ('training','source')}


def decode_surround(request,profile=None, *, width=480, height=270):
    """Lossless 2x2 atlas: front/left on top, back/right below.

    Keep the four-image transport for saved clients and training collectors.
    Each request uses its backend's exact dimensions and bounded PNG decoder.
    """
    if profile is not None and request.get('camera_profile_sha256') != profile_sha256(profile):
        raise ValueError('360 camera input profile mismatch; reload client and server')
    if 'views_atlas_png' in request:
        if 'views_png' in request or 'image_png' in request:
            raise ValueError('send one surround atlas, not duplicate camera images')
        try:
            atlas=decode_image(request['views_atlas_png'],cv2.IMREAD_COLOR,
                               max_width=2*width,max_height=2*height)
        except ValueError as error:
            raise ValueError(f'{2*width}x{2*height} bounded surround atlas required') from error
        if atlas.shape!=(2*height,2*width,3):raise ValueError(f'{2*width}x{2*height} surround atlas required')
        return [np.ascontiguousarray(atlas[y:y+height,x:x+width])
                for y,x in ((0,0),(0,width),(height,0),(height,width))]
    views=request.get('views_png')
    if not isinstance(views,list) or len(views)!=4 or views[0]!=request.get('image_png'):
        raise ValueError('four ordered camera views required; view 0 must match tracking RGB')
    frames=[decode_image(v,cv2.IMREAD_COLOR,max_width=width,max_height=height) for v in views]
    if any(f.shape!=(height,width,3) for f in frames):raise ValueError(f'{width}x{height} camera views required')
    return frames


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
    def __init__(self, encoder, site, perception, world,policy=None,training_root=None,*,max_speed_mps=10.):
        self.service=PlannerService(encoder) if encoder is not None else None
        self.fcn=perception.details.get('backend') in ('fcn512-360','fcn1024-360')
        self.fcn_input_width=int(perception.details.get('input_width',512)) if self.fcn else None
        if self.fcn and self.fcn_input_width not in (512,1024):
            raise ValueError('Unsupported FCN model input width')
        self.road_controller=policy is not None and policy.details.get('backend')=='fcn360-path-controller'
        self.model_details=perception.details.copy() if self.fcn else encoder.details
        self.site=site;self.lock=threading.Lock();self.perception=perception
        self.world=world;self.trace=[];self.session=None;self.started=time.monotonic();self.errors=0
        self.patrol=None;self.mission_id=None;self.mission=None
        self.faults=None;self.fault_origin=0.
        self.policy=policy
        self.max_speed_mps=float(getattr(policy,'max_speed_mps',max_speed_mps))
        if not .1 <= self.max_speed_mps <= 20.:
            raise ValueError('simulation maximum speed outside backend limits')
        self.camera_profile=load_profile()
        self.camera_profile_sha256=profile_sha256(self.camera_profile)
        self.training_root=Path(training_root) if training_root else ROOT/'.cache'

    def step(self, request):
        from dangjin_patrol import Patrol,track_camera_path
        if request.get('object_avoidance_enabled') or 'lidar' in request:
            raise ValueError('simulation object avoidance was removed; reload the browser')
        preview=request.get('preview',True)
        if type(preview) is not bool:raise ValueError('boolean preview flag required')
        pose=request.get('pose');validate_pose(pose)
        if request.get('requested_modes')!=[1]:raise ValueError('simulation requires explicit road mode')
        if any(k in request for k in ('selected_png','hazard_png','corridor_xy','target_xy')):
            raise ValueError('runtime accepts RGB only; client masks and ground-truth paths are forbidden')
        yaw=request.get('camera_yaw',0.)
        if type(yaw) not in (int,float) or not math.isfinite(yaw) or abs(yaw)>1.3:raise ValueError('bounded virtual-camera yaw required')
        camera_pose={**pose,'heading':pose['heading']+yaw}
        speed=request.get('speed_mps',self.max_speed_mps)
        if type(speed) not in (int,float) or not .1<=speed<=self.max_speed_mps:raise ValueError('sim speed exceeds configured maximum')
        mission_id=request.get('mission_id',request['stream_id'])
        config_fault=fault_config(request.get('sensor_faults'))
        stamp=request.get('stamp_sec')
        if type(stamp) not in (int,float) or not math.isfinite(stamp):raise ValueError('finite simulation timestamp required')
        if mission_id!=self.mission_id:
            if self.road_controller:self.policy.reset()
            self.mission_id=mission_id;self.trace=[];self.started=time.monotonic();self.errors=0
            self.patrol=Patrol(self.world['perimeter_route'],strict=self.policy is None,guidance=self.policy is None) if request.get('scenario')=='patrol' else None
            self.faults=SensorFaults(config_fault);self.fault_origin=stamp
        elif config_fault!=self.faults.config:raise ValueError('sensor fault settings changed; start a new test')
        self.mission=self.patrol.update(pose) if self.patrol else None
        if self.perception.details.get('views')==1 and any(k in request for k in ('views_png','views_atlas_png')):
            raise ValueError('front-camera mode must not send surround views')
        surround_preview=None
        if self.road_controller:
            from dangjin_fcn import apply_class_faults, binary_road_grid
            from fcn360_road import ROAD, EGO, BLIND, observed_classes
            if yaw != 0:raise ValueError('Road controller requires body-fixed surround views')
            if self.fcn:
                frames = decode_surround(request,self.camera_profile,width=self.fcn_input_width,height=self.fcn_input_width//2)
                classes, ms = self.perception.predict_classes(frames)
                detector_metrics = {'ms':ms}
            else:
                frames = decode_surround(request,self.camera_profile)
                native, detector_metrics = self.perception.predict_views(frames)
                classes = binary_road_grid(native)
            raw = float((classes[0] == ROAD).mean())
            classes, faults = apply_class_faults(classes,self.faults,stamp-self.fault_origin)
            observed = observed_classes(classes)
            road = (observed == ROAD).astype(np.uint8)
            metrics = dict(detector_metrics,views=4,raw_road_fraction=raw,road_fraction=float(road[0].mean()),
                           applied_view_road_fractions=road.mean(axis=(1,2)).tolist())
            command, action, points = self.policy.command(classes,stamp,speed)
            if preview:
                display = np.where((observed == BLIND) | (observed == 0), 255, road).astype(np.uint8)
                masks = [cv2.resize(m,(480,270),interpolation=cv2.INTER_NEAREST) for m in display]
                ego_masks = ([cv2.resize((m == EGO).astype(np.uint8),(480,270),interpolation=cv2.INTER_NEAREST)
                              for m in classes] if self.fcn else None)
                surround_preview = panorama_mask(masks,forward_heading_deg=self.camera_profile['alignment']['forward_heading_deg'],
                                                 unobserved_value=BLIND,ego_masks=ego_masks)
            return self.camera_reply(request,command,action,metrics,faults,
                                     surround_preview,road[0],points)
        if hasattr(self.perception,'predict_views'):
            camera_frames=decode_surround(request,self.camera_profile);frame=camera_frames[0]
            masks,metrics=self.perception.predict_views(camera_frames)
            masks,faults=self.faults.apply(masks,stamp-self.fault_origin)
            selected=masks[0];hazard=np.zeros_like(selected)
            if preview:surround_preview=panorama_mask(masks,forward_heading_deg=self.camera_profile['alignment']['forward_heading_deg'])
        else:
            frame=decode_image(request['image_png'],cv2.IMREAD_COLOR)
            selected,hazard,metrics=self.perception.infer(frame)
            masks,faults=self.faults.apply(selected[None],stamp-self.fault_origin);selected=masks[0]
        metrics={**metrics,'raw_road_fraction':metrics['road_fraction'],'road_fraction':float(selected.mean()),
                 'applied_view_road_fractions':[float(m.mean()) for m in masks]}
        if self.policy is not None:
            if yaw!=0:raise ValueError('neural policy requires body-fixed camera views')
            return self.neural_step(request,masks,metrics,faults,surround_preview)
        local_values=dict(request['local_config'])
        for key in ('roi_polygon','road_roi_polygon','roi_exclusion_polygon','road_roi_exclusion_polygon'):
            if local_values.get(key) is not None:local_values[key]=tuple(local_values[key])
        local=LocalPathConfig(**local_values);config=PlannerConfig(**request['planner_config'])
        planner=dict(request,selected_png=encode_png(selected),hazard_png=encode_png(hazard))
        if 'image_png' not in planner:planner['image_png']=encode_png(frame)
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
        if faults['kind']=='dropout':command=DriveDecision.stop('detection_dropout')
        if reason:command=DriveDecision.stop(reason)
        if self.mission and self.mission['complete']:command=DriveDecision.stop('lap_complete')
        self.trace.append({'stamp_sec':request['stamp_sec'],'pose':pose,'camera_yaw':yaw,'command':asdict(command),
                           'confidence':path['confidence'] if path else 0,'path_end':path['points_xy'][-1] if path else None,
                           'flow_ms':reply['timing']['flow_ms'],'surface_ms':reply['timing']['surface_ms'],
                           'perception':metrics,'sensor_faults':faults,'mission':self.mission})
        if len(self.trace)>20000:self.trace=self.trace[-20000:]
        folder=ROOT/'.cache/dangjin-perception';folder.mkdir(exist_ok=True)
        if len(self.trace)==1 or len(self.trace)%100==0 or command.vx==0:
            cv2.imwrite(str(folder/'latest-rgb.png'),frame);cv2.imwrite(str(folder/'latest-road.png'),selected*255)
        reply.update(command=asdict(command),simulation_only=True,perception=runtime_details(self.perception.details),
                     perception_metrics=metrics,mission=self.mission,
                     control='RGB path pure pursuit',no_go_zone_count=len(zones))
        reply['sensor_faults']=faults
        if preview:
            if surround_preview is not None:reply['surround_prediction_png']=encode_png(surround_preview)
            else:
                colored=np.full((*selected.shape,3),(50,48,27),np.uint8);colored[selected!=0]=(140,167,94)
                reply['prediction_png']=encode_png(colored)
        return reply

    def neural_step(self,request,masks,metrics,faults,surround_preview):
        """Camera-only learned actor shared by front and surround simulation."""
        command,action=self.policy.infer(masks)
        return self.camera_reply(request,command,action,metrics,faults,surround_preview,masks[0])

    def camera_reply(self,request,command,action,metrics,faults,surround_preview,front_mask,points=None):
        pose=request['pose']
        action['model_command']=asdict(command)
        # Episode termination is separate from a safety intervention. The
        # evaluated lap is over; do not begin a second lap automatically.
        if self.mission and self.mission['complete']:command=DriveDecision.stop('lap_complete')
        self.model_details['inferred_frames']=self.model_details.get('inferred_frames',0)+1
        entry={'stamp_sec':request['stamp_sec'],'pose':pose,'camera_yaw':0.,'command':asdict(command),
            'flow_ms':0.,'surface_ms':0.,'perception':metrics,'sensor_faults':faults,'mission':self.mission,
            'policy':action,
            'evaluation':request.get('evaluation',{}),'environment':request.get('environment',{})}
        self.trace.append(entry)
        reply={'schema':'line_tracking.connectome.v1','request_id':request['request_id'],'stream_id':request['stream_id'],
            'stamp_sec':request['stamp_sec'],'model':runtime_details(self.model_details),'perception':runtime_details(self.perception.details),
            'command':asdict(command),'policy':runtime_details(self.policy.details),'policy_metrics':action,'paths':{'1':{'points_xy':points}} if points else {},
            'timing':{'flow_ms':0.,'surface_ms':0.,'policy_ms':action['ms']},'perception_metrics':metrics,
            'mission':self.mission,'sensor_faults':faults,
            'simulation_only':True,'control':'360 Road path tracking' if self.road_controller else 'learned connectome v/omega','emergency_stop_override':False}
        if surround_preview is not None:
            reply['surround_prediction_png']=encode_png(surround_preview)
        elif request.get('preview',True):
            preview=np.full((*front_mask.shape,3),(50,48,27),np.uint8)
            preview[front_mask!=0]=(140,167,94);reply['prediction_png']=encode_png(preview)
        return reply

    def telemetry(self):
        distance=sum(math.hypot(b['pose']['x']-a['pose']['x'],b['pose']['y']-a['pose']['y']) for a,b in zip(self.trace,self.trace[1:]))
        return {'session':self.mission_id,'layout_revision':self.site.get('layout_revision'),'steps':len(self.trace),'distance_m':distance,'wall_s':time.monotonic()-self.started,
                'errors':self.errors,'latest':self.trace[-1] if self.trace else None,'mission':self.mission,
                'stops':sum(t['command']['vx']==0 for t in self.trace),'model':self.model_details,
                'perception':self.perception.details,'sensor_faults':self.faults.config if self.faults else None,
                'fault_events':len({t['sensor_faults']['event_id'] for t in self.trace if t['sensor_faults']['event_id']}),
                'policy':self.policy.details if self.policy else None,'camera_profile':self.camera_profile,
                'camera_profile_sha256':self.camera_profile_sha256,'trace':self.trace}


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
                self.reply(200,{'ready':True,'simulation_only':True,'sensor_faults_version':SensorFaults.version,'layout_revision':sim.site.get('layout_revision'),'model':sim.model_details,'no_go_zones':sim.site.get('no_go_zones',[]),'perception':sim.perception.details,'policy':sim.policy.details if sim.policy else None,'camera_transport':f'fcn-atlas-{sim.fcn_input_width}x{sim.fcn_input_width//2}' if sim.fcn else 'atlas-v2-profiled',
                    'camera_profile':sim.camera_profile,'camera_profile_sha256':sim.camera_profile_sha256,
                    'max_speed_mps':sim.max_speed_mps});return
            if self.path=='/api/telemetry':self.reply(200,sim.telemetry());return
            super().do_GET()

        def do_POST(self):
            origin=self.headers.get('Origin')
            if origin and origin not in (f'http://127.0.0.1:{port}',f'http://localhost:{port}'):
                self.reply(403,{'error':'same-origin localhost required'});return
            if self.headers.get_content_type()!='application/json':self.reply(415,{'error':'JSON required'});return
            if self.path=='/api/policy-training-frame':
                try:
                    length=int(self.headers.get('Content-Length','0'))
                    if not 0<length<=MAX_BODY:raise ValueError('bounded training frame required')
                    body=json.loads(self.rfile.read(length));index=body['index']
                    if type(index) is not int or not 0<=index<4096:raise ValueError('invalid policy sample index')
                    validate_pose(body.get('pose'))
                    frames=decode_surround(body,sim.camera_profile)
                    folder=sim.training_root/'dangjin-policy-training';folder.mkdir(parents=True,exist_ok=True)
                    for sector,frame in enumerate(frames):
                        cv2.imwrite(str(folder/f'{index:04d}-{sector}-rgb.png'),frame)
                    (folder/f'{index:04d}.json').write_text(json.dumps({'index':index,'pose_group':body['pose_group'],
                        'pose':body['pose'],'route_index':body['route_index'],'local_config':body['local_config'],
                        'layout_revision':sim.site.get('layout_revision'),'camera_pitch_deg':body.get('camera_pitch_deg',0),
                        'camera_profile_sha256':sim.camera_profile_sha256,'camera_profile':sim.camera_profile,
                        'label_use':'offline policy training only'},allow_nan=False)+'\n')
                    self.reply(200,{'saved':index})
                except Exception as error:self.reply(422,{'error':str(error)})
                return
            if self.path=='/api/training-frame':
                try:
                    length=int(self.headers.get('Content-Length','0'))
                    if not 0<length<=MAX_BODY:raise ValueError('bounded training frame required')
                    body=json.loads(self.rfile.read(length));index=body['index']
                    surround=body.get('dataset')=='connectome360'
                    if surround and body.get('camera_profile_sha256')!=sim.camera_profile_sha256:
                        raise ValueError('360 training camera profile mismatch')
                    if type(index) is not int or not 0<=index<(512 if surround else 256):raise ValueError('invalid sample index')
                    frame=decode_image(body['image_png'],cv2.IMREAD_COLOR);mask=decode_image(body['selected_png'],cv2.IMREAD_GRAYSCALE)
                    if mask.shape!=frame.shape[:2] or not np.isin(mask,[0,1]).all():raise ValueError('aligned binary training labels required')
                    folder=sim.training_root/('dangjin-connectome-training' if surround else 'dangjin-training');folder.mkdir(parents=True,exist_ok=True)
                    cv2.imwrite(str(folder/f'{index:03d}-rgb.png'),frame);cv2.imwrite(str(folder/f'{index:03d}-road.png'),mask)
                    if surround:
                        validate_pose(body.get('pose'))
                        (folder/f'{index:03d}.json').write_text(json.dumps({'index':index,'pose_group':index//4,'sector':index%4,
                            'pose':body['pose'],'layout_revision':sim.site.get('layout_revision'),'camera_pitch_deg':body.get('camera_pitch_deg',0),
                            'camera_profile_sha256':sim.camera_profile_sha256,'camera_profile':sim.camera_profile,
                            'label_use':'offline training only'},allow_nan=False)+'\n')
                    self.reply(200,{'saved':index})
                except Exception as error:self.reply(422,{'error':str(error)})
                return
            if self.path!='/api/step':self.reply(404,{'error':'unknown endpoint'});return
            if not sim.lock.acquire(blocking=False):self.reply(409,{'error':'planner busy; no queued control commands'});return
            try:
                length=int(self.headers.get('Content-Length','0'))
                if not 0<length<=(FCN_MAX_BODY if sim.fcn else MAX_BODY):raise ValueError('bounded request required')
                self.reply(200,sim.step(json.loads(self.rfile.read(length))))
            except Exception as error:
                sim.errors+=1
                if sim.road_controller:sim.policy.reset()
                else:sim.service.encoder.reset()
                self.reply(422,{'error':str(error)[:240],'command':asdict(DriveDecision.stop('planner_error'))})
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
    p.add_argument('--control',choices=['neural','baseline','road-controller'],default='neural')
    p.add_argument('--fcn-model',type=Path,help='fine-tuned FCN360 folder with ONNX and complete.json')
    p.add_argument('--max-speed',type=float,default=10.,help='simulation maximum m/s (default 10.0, range 0.1 to 20.0)')
    p.add_argument('--policy-model',type=Path,default=ROOT/'.cache/dangjin-connectome-policy-vision')
    p.add_argument('--front-model',type=Path,help='trained single-front-camera model package')
    p.add_argument('--training-root',type=Path,help='separate directory for newly rendered training data')
    p.add_argument('--alias-port',type=int);a=p.parse_args()
    if not .1 <= a.max_speed <= 20.:p.error('--max-speed must be 0.1 to 20.0 m/s')
    if a.fcn_model and (a.front_model or a.control=='baseline' or a.road_model):p.error('select a single Road backend')
    if a.control=='road-controller' and (a.front_model or a.road_backend!='connectome'):
        p.error('road-controller requires a 360-degree Connectome detector')
    from connectome_flow import FlyvisFlow
    directory=ROOT/'environments/dangjin';site=json.loads((directory/'site.json').read_text())
    cv2.setNumThreads(2)
    if a.fcn_model:
        from dangjin_fcn import FcnPerception,FcnDrive
        detector=FcnPerception(a.fcn_model)
        drive=FcnDrive(load_profile()['mount']['height_m'],a.max_speed)
        sim=SimulationService(None,site,detector,json.loads((directory/'world.json').read_text()),drive)
        server=make_sim_server(sim,directory,a.port)
        print(f'Ready: http://127.0.0.1:{a.port} · FCN360 · {a.max_speed} m/s · simulation only',flush=True)
        server.serve_forever()
        return
    if a.front_model:
        from front_connectome import FrontConnectome,FrontSimulationPerception,FrontSimulationPolicy
        model=FrontConnectome(a.front_model,encoder_device=a.device,head_device=a.road_device)
        sim=SimulationService(model.encoder,site,FrontSimulationPerception(model),
            json.loads((directory/'world.json').read_text()),FrontSimulationPolicy(model,max_speed_mps=a.max_speed),a.training_root)
        server=make_sim_server(sim,directory,a.port)
        print(f'Ready: http://127.0.0.1:{a.port} · single front camera · simulation only',flush=True)
        server.serve_forever()
        return
    encoder=FlyvisFlow(a.bundle,a.sha256,a.device)
    if a.road_backend=='connectome':
        from connectome_road import ConnectomeRoadPerception
        perception=ConnectomeRoadPerception(encoder,a.road_model or ROOT/'.cache/dangjin-connectome-road',a.road_device)
    else:
        from dangjin_perception import RoadPerception
        perception=RoadPerception(a.road_model or ROOT/'.cache/dangjin-road-model',a.road_device)
    policy=None
    if a.control=='road-controller':
        from dangjin_fcn import FcnDrive
        policy=FcnDrive(load_profile()['mount']['height_m'],a.max_speed)
    elif a.control=='neural':
        if a.road_backend!='connectome':raise ValueError('neural drive requires Flyvis perception')
        from connectome_policy import ConnectomeDrivePolicy
        policy=ConnectomeDrivePolicy(a.policy_model,perception,'cpu',max_speed_mps=a.max_speed)
        if policy.details['training']['layout_revision']!=site['layout_revision']:raise ValueError('policy belongs to another map')
    sim=SimulationService(encoder,site,perception,json.loads((directory/'world.json').read_text()),policy,a.training_root,max_speed_mps=a.max_speed)
    if a.alias_port is not None:
        alias=make_sim_server(sim,directory,a.alias_port)
        threading.Thread(target=alias.serve_forever,daemon=True).start()
    server=make_sim_server(sim,directory,a.port)
    print(f'Ready: http://127.0.0.1:{a.port} · 360 camera · {a.control} · simulation only',flush=True)
    server.serve_forever()


if __name__=='__main__':main()

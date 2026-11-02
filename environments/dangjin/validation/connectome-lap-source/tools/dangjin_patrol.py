"""Ordered lap accounting and camera-path tracking, independent of perception."""
import math
import numpy as np
from swin_l_drive_control import DriveDecision


def wrap(angle):return (angle+math.pi)%(2*math.pi)-math.pi


class Patrol:
    def __init__(self,route):
        self.points=np.asarray(route,float)
        self.s=np.r_[0,np.cumsum(np.linalg.norm(np.diff(self.points,axis=0),axis=1))]
        self.length=float(self.s[-1]);self.progress=0.;self.distance=0.;self.last=None;self.visited=set();self.complete=False
        tangent=self.points[min(12,len(self.points)-2)]-self.points[0]
        self.tangent=tangent/np.linalg.norm(tangent)

    def update(self,pose):
        p=np.array([pose['x'],pose['y']])
        previous=self.last
        if self.last is not None:
            step=float(np.linalg.norm(p-self.last))
            if step>1.5:raise ValueError('patrol pose jump; reset required')
            self.distance+=step
        elif np.linalg.norm(p-self.points[0])>2:raise ValueError('patrol must start at its start gate')
        self.last=p
        indices=np.flatnonzero((self.s>=self.progress-3)&(self.s<=self.progress+6))
        nearest=indices[np.argmin(np.linalg.norm(self.points[indices]-p,axis=1))]
        cross=float(np.linalg.norm(self.points[nearest]-p))
        if cross>4:raise ValueError('patrol left its route neighbourhood')
        self.progress=max(self.progress,float(self.s[nearest]));self.visited.add(int(self.progress//20))
        # Ordered nearest search + bounded movement prevents counting a shortcut.
        expected=set(range(int(self.length//20)))
        relative=p-self.points[0];along=float(relative@self.tangent)
        lateral=float(relative@np.array([-self.tangent[1],self.tangent[0]]))
        crossed=bool(previous is not None and float((previous-self.points[0])@self.tangent)<0<=along)
        # A lap ends by crossing the start's transverse gate, not by hitting
        # an exact XY point. The 5 m gate fits inside this 8 m road, including
        # the 0.65 m footprint; ordered progress prevents early completion.
        self.complete=bool(self.complete or (self.progress>self.length-3 and self.distance>.94*self.length
                       and crossed and abs(lateral)<=2.5 and expected<=self.visited))
        target_s=(self.progress+6)%self.length
        target=[float(np.interp(target_s,self.s,self.points[:,i])) for i in (0,1)]
        return {'progress_m':self.progress,'length_m':self.length,'distance_m':self.distance,
                'fraction':1. if self.complete else min(.999,self.progress/self.length),'complete':self.complete,
                'checkpoints':len(self.visited),'target_site':target,'cross_track_m':cross,
                'return_gate_crossed':crossed,'return_gate_lateral_m':lateral,'return_gate_half_width_m':2.5}


def track_camera_path(points,view_yaw,max_speed=.8):
    """Pure pursuit of a currently observed path, never of the map waypoint."""
    if points is None or len(points)<2:return DriveDecision.stop('path_unavailable')
    p=np.asarray(points,float);c,s=math.cos(view_yaw),math.sin(view_yaw)
    p=p@np.array([[c,s],[-s,c]])
    if not np.isfinite(p).all():return DriveDecision.stop('invalid_path')
    distance=np.linalg.norm(p,axis=1)
    # Short lookahead follows tight outer bends without driving across the chord.
    i=int(np.argmin(abs(distance-3.5)));x,y=p[i]
    heading=math.atan2(y,x)
    if x<=0 or abs(heading)>math.radians(75):return DriveDecision.stop('lateral_target')
    curvature=2*y/max(x*x+y*y,1.)
    speed=min(max_speed,.30/max(abs(curvature),.01))
    speed=max(.15,speed)
    yaw=float(np.clip(speed*curvature,-.35,.35))
    return DriveDecision(float(speed),0.,yaw,'tracking_slow_turn' if speed<max_speed-.01 else 'tracking')

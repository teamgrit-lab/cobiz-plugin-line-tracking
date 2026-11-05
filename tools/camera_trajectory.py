"""Camera-path command proposals shared by offline data preparation."""
import math
import numpy as np
from drive_control import DriveDecision

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

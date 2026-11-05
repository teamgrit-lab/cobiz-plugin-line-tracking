"""Shared input contract for recorded /dev/video91 panoramas and the simulator.

No model loading, camera acquisition, ROS or robot control. Input is already
stitched equirectangular BGR; output order is robot front, left, rear, right.
"""
from functools import lru_cache
import hashlib
import json
from pathlib import Path

import numpy as np

PROFILE_PATH = Path(__file__).resolve().parents[1]/'config/camera360-profile.json'


def validate_profile(profile):
    if profile.get('schema') != 'connectum.camera360.v1':
        raise ValueError('unsupported 360 camera profile')
    source, views, alignment = profile['source'], profile['views'], profile['alignment']
    if (source['projection'] != 'equirectangular' or source['width'] != 2880 or source['height'] != 1440
            or (views['width'], views['height'], views['vertical_fov_deg']) != (480,270,90)
            or views['order'] != ['front','left','rear','right']
            or views['interpolation'] != 'bilinear-srgb' or views['pixel_centers'] != 'half-pixel'
            or alignment['positive_heading'] != 'left'):
        raise ValueError('incompatible 360 camera projection contract')
    for value in [alignment['forward_heading_deg'], *[profile['mount'][k] for k in ['height_m','pitch_deg','roll_deg']]]:
        if type(value) not in (int,float) or not np.isfinite(value):
            raise ValueError('finite camera alignment required')
    if not 0 < profile['mount']['height_m'] < 5:
        raise ValueError('invalid camera mounting height')
    return profile


def load_profile(path=PROFILE_PATH):
    return validate_profile(json.loads(Path(path).read_text()))


def profile_sha256(profile):
    # JSON.stringify emits 2 for 2.0; preserve that convention when a measured
    # mounting value is later edited as an integral float in the JSON profile.
    def canonical(value):
        if isinstance(value,dict):return {k:canonical(v) for k,v in value.items()}
        if isinstance(value,list):return [canonical(v) for v in value]
        if isinstance(value,float) and value.is_integer():return int(value)
        return value
    return hashlib.sha256(json.dumps(canonical(profile),sort_keys=True,separators=(',',':'),ensure_ascii=False).encode()).hexdigest()


@lru_cache(maxsize=8)
def projection_maps(forward_heading_deg=90):
    yy, xx = np.mgrid[:270,:480]
    right, up = (xx-239.5)/135., (134.5-yy)/135.
    lat = np.arctan2(up,np.sqrt(1+right**2))
    maps = []
    for sector in range(4):
        angle = np.deg2rad(forward_heading_deg+sector*90)
        lon = (np.arctan2(right,1)-angle+np.pi) % (2*np.pi)-np.pi
        x = ((lon/(2*np.pi)+.5)*2880-.5).astype(np.float32)
        y = ((.5-lat/np.pi)*1440-.5).astype(np.float32)
        x.setflags(write=False);y.setflags(write=False);maps.append((x,y))
    return tuple(maps)


def project_panorama(image,profile=None):
    profile=validate_profile(profile or load_profile())
    if image.dtype != np.uint8 or image.shape != (1440,2880,3):
        raise ValueError('2880x1440 uint8 BGR stitched panorama required; do not stretch or crop it')
    frames=[]
    for x,y in projection_maps(profile['alignment']['forward_heading_deg']):
        # Float bilinear sampling in encoded sRGB, matching the GPU texture path.
        # OpenCV INTER_LINEAR quantizes the maps to 1/32 pixels, so do not use it here.
        x0=np.floor(x).astype(np.int32);y0=np.floor(y).astype(np.int32)
        ax=(x-x0)[...,None];ay=(y-y0)[...,None]
        xa=x0%2880;xb=(x0+1)%2880;ya=y0.clip(0,1439);yb=(y0+1).clip(0,1439)
        upper=image[ya,xa]*(1-ax)+image[ya,xb]*ax
        lower=image[yb,xa]*(1-ax)+image[yb,xb]*ax
        frames.append(np.floor(upper*(1-ay)+lower*ay+.5).clip(0,255).astype(np.uint8))
    return frames


def view_atlas(frames):
    if len(frames)!=4 or any(f.shape!=(270,480,3) for f in frames):
        raise ValueError('four 480x270 views required')
    return np.concatenate([np.concatenate(frames[:2],axis=1),np.concatenate(frames[2:],axis=1)],axis=0)

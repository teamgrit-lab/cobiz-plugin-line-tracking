"""Training-only, deterministic illumination augmentation shared by 360 views.

The attenuation field wraps horizontally on a panorama. Corresponding rays in
overlapping views therefore receive the same shadow. Geometry/labels stay fixed.
No augmentation is applied at normal inference time.
"""
import hashlib
from functools import lru_cache

import cv2
import numpy as np

from surround_camera import projection_maps

SCHEMA = 'connectome360.shadow_augmentation.v1'


def parameters(seed, key, variant=0):
    value = hashlib.sha256(f'{seed}|{key}|{variant}'.encode()).digest()
    rng = np.random.default_rng(int.from_bytes(value[:8], 'little'))
    strong = variant % 3 == 1
    polygons = []
    for sector in rng.choice(4, int(rng.integers(2, 5)), replace=False):
        cx = (sector + rng.uniform(.15, .85)) * 180
        cy = rng.uniform(218, 300)
        width, height = rng.uniform(85, 215), rng.uniform(90, 200)
        theta = rng.uniform(-.8, .8)
        points = np.array([[-.5,-.5],[.5,-.4],[.6,.5],[-.4,.5]]) * [width,height]
        rotation = np.array([[np.cos(theta),-np.sin(theta)],[np.sin(theta),np.cos(theta)]])
        polygons.append((points @ rotation.T + [cx,cy]).round(3).tolist())
    return dict(schema=SCHEMA,seed=int(seed),key=str(key),variant=int(variant),
                exposure=float(rng.uniform(.70 if strong else .86, 1.20)),
                gamma=float(rng.uniform(.78, 1.30 if strong else 1.16)),
                strength=float(rng.uniform(.40 if strong else .23, .75 if strong else .58)),
                softness=float(rng.uniform(.5, 5.5)), polygons_xy=polygons)


def field(params):
    mask = np.zeros((360,720),np.float32)
    for polygon in params['polygons_xy']:
        points = np.asarray(polygon,np.float32)
        for offset in [-720,0,720]:
            cv2.fillPoly(mask,[(points+[offset,0]).round().astype(np.int32)],1.)
    tiled = np.tile(mask,(1,3))
    blurred = cv2.GaussianBlur(tiled,(0,0),params['softness'])[:,720:1440]
    ground = np.clip((np.arange(360,dtype=np.float32)/360-.44)/.08,0,1)[:,None]
    return (1.-params['strength']*blurred*ground).astype(np.float32)


@lru_cache(maxsize=1)
def maps():
    return tuple((((x+.5)/4-.5).astype(np.float32),((y+.5)/4-.5).astype(np.float32))
                 for x,y in projection_maps(90))


def augment_views(frames, *, seed, key, variant=0, view_indices=None):
    indices = list(range(len(frames))) if view_indices is None else list(view_indices)
    if len(indices)!=len(frames) or any(i not in range(4) for i in indices):
        raise ValueError('Aligned 360 direction indices required')
    params = parameters(seed,key,variant)
    attenuation = field(params)
    result = []
    for image,index in zip(frames,indices):
        if image.shape!=(270,480,3) or image.dtype!=np.uint8:
            raise ValueError('480x270 uint8 BGR training image required')
        gain = cv2.remap(attenuation,*maps()[index],cv2.INTER_LINEAR,borderMode=cv2.BORDER_WRAP)
        changed = np.power(image.astype(np.float32)/255.,params['gamma'])
        changed *= gain[...,None]*params['exposure']
        result.append(np.rint(changed*255).clip(0,255).astype(np.uint8))
    return result,params

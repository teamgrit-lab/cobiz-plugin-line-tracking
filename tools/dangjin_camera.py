"""Projection of four camera masks for the synthetic 360-degree camera."""
import cv2
import numpy as np
from functools import lru_cache


@lru_cache(maxsize=4)
def _panorama_projection(width,height,forward_heading_deg=0):
    """Projection depends only on display size, never on the current masks."""
    yy,xx=np.mgrid[:height,:width]
    lon=((xx+.5)/width-.5)*2*np.pi
    lat=(.5-(yy+.5)/height)*np.pi
    # Panorama longitude increases right; positive robot heading turns left.
    heading=-lon-np.deg2rad(forward_heading_deg)
    sector=np.mod(np.rint(heading/(np.pi/2)).astype(int),4)
    delta=(heading-sector*np.pi/2+np.pi)%(2*np.pi)-np.pi
    u=239.5-135*np.tan(delta)
    v=134.5-135*np.tan(lat)/np.cos(delta)
    visible=(u>=0)&(u<480)&(v>=0)&(v<270)
    return u.astype(np.float32),v.astype(np.float32),visible,tuple(visible&(sector==i) for i in range(4))


def panorama_mask(masks,width=768,height=384,forward_heading_deg=0):
    """Four ±45° vertical views cover every azimuth; unseen polar caps stay unknown."""
    if len(masks)!=4 or any(m.shape!=(270,480) for m in masks):raise ValueError('four aligned masks required')
    u,v,visible,sectors=_panorama_projection(width,height,forward_heading_deg)
    output=np.full((height,width,3),(50,48,27),np.uint8)
    output[~visible]=(85,85,85)
    for i,mask in enumerate(masks):
        sampled=cv2.remap(mask,u,v,cv2.INTER_NEAREST,borderMode=cv2.BORDER_CONSTANT)
        output[sectors[i]&(sampled!=0)]=(140,167,94)
    return output

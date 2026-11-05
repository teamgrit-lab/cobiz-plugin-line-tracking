"""Bounded lossless camera atlas codec for saved replay and training inputs."""
import base64
import cv2
import numpy as np
from connectome_server import decode_image
from surround_camera import profile_sha256

def encode_png(image):
    return base64.b64encode(cv2.imencode('.png',image,[cv2.IMWRITE_PNG_COMPRESSION,1])[1]).decode()

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

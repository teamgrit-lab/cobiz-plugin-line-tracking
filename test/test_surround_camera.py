"""Independent direction landmarks and JS/Python camera-contract checks."""
import copy
import json
from pathlib import Path
import subprocess
import sys

import numpy as np
import pytest

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
from surround_camera import load_profile,profile_sha256,project_panorama,projection_maps,view_atlas
from dangjin_camera import _panorama_projection
from dangjin_sim_server import decode_surround,encode_png


def test_landmarks_are_front_left_rear_right_without_mirroring():
    pano=np.zeros((1440,2880,3),np.uint8)
    # Device image positions, independently labelled from the user's forward=90°.
    for column,color in [(720,[255,0,0]),(0,[0,255,0]),(2160,[0,0,255]),(1440,[255,255,0])]:
        columns=np.arange(column-40,column+40)%2880
        pano[:,columns]=color
    views=project_panorama(pano)
    for frame,color in zip(views,[[255,0,0],[0,255,0],[0,0,255],[255,255,0]]):
        np.testing.assert_array_equal(frame[135,240],color)
    # Left/right in an individual view retain increasing source-image x.
    x,y=projection_maps()[0]
    assert x[135,241]>x[135,239] and y[136,240]>y[134,240]


def test_bilinear_sampling_wraps_longitude_and_preserves_constant_colors():
    image=np.full((1440,2880,3),[17,123,246],np.uint8)
    for view in project_panorama(image):assert np.all(view==[17,123,246])
    left_map=projection_maps()[1][0]
    assert np.any(left_map<0) or (left_map.min()<5 and left_map.max()>2875)
    atlas=view_atlas(project_panorama(image))
    assert atlas.shape==(540,960,3)


def test_source_size_and_input_profile_cannot_silently_change():
    with pytest.raises(ValueError,match='do not stretch'):
        project_panorama(np.zeros((720,1440,3),np.uint8))
    profile=load_profile();other=copy.deepcopy(profile);other['alignment']['forward_heading_deg']=0
    assert profile_sha256(profile)!=profile_sha256(other)
    atlas=encode_png(np.zeros((540,960,3),np.uint8))
    with pytest.raises(ValueError,match='profile mismatch'):
        decode_surround({'views_atlas_png':atlas},profile)
    with pytest.raises(ValueError,match='profile mismatch'):
        decode_surround({'views_atlas_png':atlas,'camera_profile_sha256':profile_sha256(other)},profile)
    assert len(decode_surround({'views_atlas_png':atlas,'camera_profile_sha256':profile_sha256(profile)},profile))==4
    other['mount']['height_m']=2.
    integral=copy.deepcopy(other);integral['mount']['height_m']=2
    assert profile_sha256(other)==profile_sha256(integral)


def test_mask_preview_uses_the_same_raw_panorama_direction():
    _,_,visible,sectors=_panorama_projection(768,384,90)
    # x=width/4 is forward; centre is robot-right, matching device imagery.
    assert visible[192,192] and sectors[0][192,192]
    assert sectors[3][192,384] and sectors[2][192,576] and sectors[1][192,0]


def test_browser_and_python_agree_on_hash_and_subpixel_sampling_coordinates():
    script="""
      import fs from 'node:fs';import {pathToFileURL} from 'node:url';
      const root=process.cwd(),src=fs.readFileSync('environments/dangjin/camera360.js','utf8')
        .replace("from 'three'",`from '${pathToFileURL(root+'/environments/dangjin/vendor/three.module.js')}'`);
      const m=await import('data:text/javascript;base64,'+Buffer.from(src).toString('base64'));
      const p=JSON.parse(fs.readFileSync('environments/dangjin/camera360-profile.json'));
      const points=[];for(let s=0;s<4;s++)for(const [x,y] of [[0,0],[479,269],[240,135],[12,98]])
        points.push({s,x,y,source:m.sourcePixelForView(p,s,x,y)});
      console.log(JSON.stringify({hash:await m.camera360ProfileHash(p),points}));
    """
    result=subprocess.run(['node','--input-type=module','-e',script],cwd=ROOT,capture_output=True,text=True,check=True)
    value=json.loads(result.stdout);assert value['hash']==profile_sha256(load_profile())
    for point in value['points']:
        x,y=projection_maps()[point['s']]
        np.testing.assert_allclose(point['source'],[x[point['y'],point['x']],y[point['y'],point['x']]],atol=.0003,rtol=0)

#!/usr/bin/env python3
"""Rasterize the browser's exported road geometry, never a GPS/SLAM map."""
import argparse
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image

ROOT=Path(__file__).resolve().parents[1]/'environments/dangjin'


def raster(world, resolution=.25, radius=0.):
    points=np.concatenate([r['samples'] for r in world['roads']])
    lo=np.floor((points.min(axis=0)-15)/resolution)*resolution
    hi=np.ceil((points.max(axis=0)+15)/resolution)*resolution
    width,height=np.ceil((hi-lo)/resolution).astype(int)
    free=np.zeros((height,width),bool)
    def window(xmin,ymin,xmax,ymax):
        left=max(0,int(math.floor((xmin-lo[0])/resolution)))
        right=min(width,int(math.ceil((xmax-lo[0])/resolution)))
        top=max(0,int(math.floor((hi[1]-ymax)/resolution)))
        bottom=min(height,int(math.ceil((hi[1]-ymin)/resolution)))
        x=lo[0]+(np.arange(left,right)+.5)*resolution
        y=hi[1]-(np.arange(top,bottom)+.5)*resolution
        return (slice(top,bottom),slice(left,right)),x[None,:],y[:,None]
    for road in world['roads']:
        clearance=road['width']/2-radius
        if clearance<=0:continue
        for a,b in zip(road['samples'],road['samples'][1:]):
            a,b=np.asarray(a),np.asarray(b);delta=b-a
            sl,x,y=window(*(np.minimum(a,b)-clearance),*(np.maximum(a,b)+clearance))
            t=np.clip(((x-a[0])*delta[0]+(y-a[1])*delta[1])/max(float(delta@delta),1e-12),0,1)
            free[sl]|=(x-a[0]-t*delta[0])**2+(y-a[1]-t*delta[1])**2<=clearance**2
    for box in world['obstacles']+world['no_go_zones']:
        x0,y0=box['x'],box['y'];x1,y1=x0+box['length'],y0+box['width']
        # Occupy every cell intersecting a support, even when a 13 cm pole
        # is narrower than the 25 cm grid and misses every cell centre.
        half=resolution/2
        sl,x,y=window(x0-radius-half,y0-radius-half,x1+radius+half,y1+radius+half)
        dx=np.maximum(np.maximum(x0-x-half,0),x-x1-half);dy=np.maximum(np.maximum(y0-y-half,0),y-y1-half)
        free[sl]&=dx**2+dy**2>radius**2
    return free,lo


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,default=ROOT)
    args=parser.parse_args();world=json.loads((args.root/'world.json').read_text())
    out=args.root/'navigation';out.mkdir(exist_ok=True)
    for name,radius in [('occupancy',0.),('robot-clearance',world['robot_radius_m'])]:
        free,origin=raster(world,radius=radius)
        Image.fromarray(np.where(free,254,0).astype(np.uint8)).save(out/f'{name}.pgm')
        (out/f'{name}.yaml').write_text(f'image: {name}.pgm\nmode: trinary\nresolution: 0.25\norigin: [{origin[0]:.2f}, {origin[1]:.2f}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
        if radius:
            rgb=np.empty((*free.shape,3),np.uint8);rgb[:]=[233,231,222];rgb[free]=[52,105,91]
            Image.fromarray(rgb).save(out/'drivable-preview.png')
    metadata={'schema':'dangjin.navigation.v1','frame':'site_xy','registered_to_gps':False,
              'resolution_m':.25,'robot_radius_m':world['robot_radius_m'],
              'occupancy':'unbuffered solid footprints + no-go + outside roads',
              'robot-clearance':'already inflated for radius; do not inflate a second time',
              'limitations':'planar approximate layout; conservative solid barn footprints; no ground elevation/contact physics',
              'source':'../world.json'}
    (out/'metadata.json').write_text(json.dumps(metadata,ensure_ascii=False,indent=2)+'\n')
    print(f'Exported navigation grids to {out}')


if __name__=='__main__':main()

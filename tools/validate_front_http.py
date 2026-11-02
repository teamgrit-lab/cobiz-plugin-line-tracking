#!/usr/bin/env python3
"""Exercise the real loopback model service and plugin adapter without ROS."""
import argparse
import json
from pathlib import Path
import urllib.error
import urllib.request
import cv2
import numpy as np
from front_connectome_client import FrontConnectomeSegmenter
from connectome_server import encode_image
from front_connectome_service import PROTOCOL


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--model',type=Path,required=True)
    p.add_argument('--url',default='http://127.0.0.1:8092/v1/front-road')
    a=p.parse_args();config=json.loads((a.model/'model.json').read_text())
    sha=config['weights_sha256']['front_model.pt']
    client=FrontConnectomeSegmenter(a.url,sha,5.)
    row=next(r for r in json.loads((a.model/'training-samples.json').read_text()) if r['source']=='real_pseudo' and r['split']=='val')
    image=cv2.imread(row['image']);r=client.segment(image,source_stamp=1.)
    s=client.segment(cv2.cvtColor(image,cv2.COLOR_BGR2RGB),color_order='rgb',source_stamp=2.)
    assert np.array_equal(r.selected_mask,s.selected_mask) and r.front_command==s.front_command
    try:FrontConnectomeSegmenter(a.url,'0'*64,5.)
    except ValueError:pass
    else:raise AssertionError('bad model hash accepted')
    try:client.segment(image,source_stamp=2.)
    except urllib.error.HTTPError as e:assert e.code==422
    else:raise AssertionError('duplicate camera stamp accepted')
    req=dict(schema=PROTOCOL,stream_id='invalid',request_id='1',stamp_sec=1.,mode=1,image_png=encode_image(image),pose={'x':0})
    try:urllib.request.urlopen(urllib.request.Request(a.url,data=json.dumps(req).encode(),headers={'Content-Type':'application/json'}))
    except urllib.error.HTTPError as e:assert e.code==422
    else:raise AssertionError('pose input accepted')
    report={'passed':True,'sample':row['id'],'command':r.front_command,'model_sha256':sha,
        'rgb_bgr_identical':True,'wrong_hash_rejected':True,'duplicate_stamp_rejected':True,'pose_input_rejected':True,
        'first_request_latency_seconds':r.total_seconds,'second_request_latency_seconds':s.total_seconds,
        'scope':'actual model HTTP and plugin adapter, no ROS/Sport output'}
    (a.model/'validation').mkdir(exist_ok=True)
    (a.model/'validation/http-adapter.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))


if __name__=='__main__':main()

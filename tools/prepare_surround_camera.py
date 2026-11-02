#!/usr/bin/env python3
"""Convert one real stitched 360 JPEG/PNG to the simulator's exact input convention."""
import argparse
import base64
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
from surround_camera import PROFILE_PATH,load_profile,profile_sha256,project_panorama,view_atlas


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('image',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--profile',type=Path,default=PROFILE_PATH)
    args=parser.parse_args()
    profile=load_profile(args.profile)
    original_atlas=None
    if args.image.suffix.lower()=='.json':
        capture=json.loads(args.image.read_text())
        if capture.get('schema')!='dangjin.camera360.capture.v1' or capture.get('camera_profile_sha256')!=profile_sha256(profile):
            parser.error('simulator capture profile does not match selected camera profile')
        frame=cv2.imdecode(np.frombuffer(base64.b64decode(capture['panorama_png'],validate=True),np.uint8),cv2.IMREAD_COLOR)
        original_atlas=cv2.imdecode(np.frombuffer(base64.b64decode(capture['views_atlas_png'],validate=True),np.uint8),cv2.IMREAD_COLOR)
    else:frame=cv2.imread(str(args.image),cv2.IMREAD_COLOR|cv2.IMREAD_IGNORE_ORIENTATION)
    if frame is None:parser.error('cannot decode input image')
    views=project_panorama(frame,profile)
    args.output.mkdir(parents=True,exist_ok=True)
    for name,view in zip(profile['views']['order'],views):
        cv2.imwrite(str(args.output/f'{name}.png'),view)
    cv2.imwrite(str(args.output/'views-atlas.png'),view_atlas(views))
    cv2.imwrite(str(args.output/'panorama.png'),frame)
    (args.output/'camera-profile.json').write_text(json.dumps(profile,indent=2)+'\n')
    report={'schema':'connectum.camera360.prepared.v1','source':str(args.image.resolve()),
            'source_sha256':hashlib.sha256(args.image.read_bytes()).hexdigest(),
            'camera_profile_sha256':profile_sha256(profile),'camera_profile_id':profile['id'],
            'view_order':profile['views']['order'],'source_headings_deg':[(profile['alignment']['forward_heading_deg']+i*90)%360 for i in range(4)],
            'color_order':'BGR internally; RGB PNG files','inference_run':False,'robot_control':False}
    if original_atlas is not None:
        if original_atlas.shape!=(540,960,3):parser.error('invalid simulator atlas dimensions')
        error=np.abs(original_atlas.astype(np.int16)-view_atlas(views).astype(np.int16))
        report['gpu_vs_cpu']={'mean_absolute_error_255':float(error.mean()),'max_absolute_error_255':int(error.max()),
                             'fraction_channels_within_2':float((error<=2).mean())}
        cv2.imwrite(str(args.output/'simulator-atlas.png'),original_atlas)
    (args.output/'input-report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,ensure_ascii=False))


if __name__=='__main__':main()

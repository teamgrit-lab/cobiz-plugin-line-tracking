#!/usr/bin/env python3
"""Four-view Flyvis with direct USB capture. No image transport or motion publisher."""
import argparse
import json
import os

import numpy as np

from dataclasses import asdict
from connectome360_runtime import Connectome360Runtime
from direct_camera_service import DirectCameraService, UsbSource, serve
from surround_camera import load_profile, project_panorama


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--model', required=True)
    p.add_argument('--sha256', default=os.environ.get('CONNECTUM_360_MODEL_SHA256', ''))
    p.add_argument('--device', choices=['cpu', 'cuda'], default='cuda')
    p.add_argument('--head-device', choices=['cpu', 'cuda', 'mps'])
    p.add_argument('--port', type=int, default=8093)
    p.add_argument('--video-device', default='/dev/video91')
    args = p.parse_args()
    model = Connectome360Runtime(args.model, args.sha256, encoder_device=args.device,
                                head_device=args.head_device or args.device)
    model.infer_views([np.full((270, 480, 3), 127, np.uint8) for _ in range(4)])
    model.reset()
    print(json.dumps(dict(ready=True, model=model.details)), flush=True)
    profile = load_profile()
    def predict(panorama):
        _, command, metrics = model.infer_views(project_panorama(panorama, profile))
        return dict(command=asdict(command), metrics=metrics)
    serve(DirectCameraService(model, UsbSource(args.video_device), predict), args.port)


if __name__ == '__main__':
    main()

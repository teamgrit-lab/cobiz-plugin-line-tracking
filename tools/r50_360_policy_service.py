#!/usr/bin/env python3
"""Camera-owned R50 perception + retained 360 actor. No motion publisher."""
import argparse
from dataclasses import asdict
import json
import os

import numpy as np

from direct_camera_service import DirectCameraService, UsbSource, serve
from r50_360_policy_runtime import R50Policy360Runtime
from surround_camera import load_profile, project_panorama


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--detector', required=True)
    parser.add_argument('--detector-sha256', default=os.environ.get('R50_360_MODEL_SHA256', ''))
    parser.add_argument('--policy-package', required=True)
    parser.add_argument('--policy-sha256', default=os.environ.get('CONNECTUM_360_MODEL_SHA256', ''))
    parser.add_argument('--device', choices=['cpu', 'cuda'], default='cuda')
    parser.add_argument('--port', type=int, default=8093)
    parser.add_argument('--video-device', default='/dev/video91')
    args = parser.parse_args()
    model = R50Policy360Runtime(args.detector, args.detector_sha256, args.policy_package,
                                args.policy_sha256, device=args.device)
    # Warm weights before serving, without opening the camera or reading real
    # images. Only an accepted Road request can acquire the USB device.
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

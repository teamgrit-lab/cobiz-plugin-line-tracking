#!/usr/bin/env python3
"""Four-view Flyvis inference with optional direct USB capture. No motion publisher."""
import argparse
import json
import os

import numpy as np

from connectome360_runtime import Connectome360Runtime, PROTOCOL
from front_connectome_service import FrontService, make_server


class Connectome360Service(FrontService):
    protocol = PROTOCOL


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--model', required=True)
    p.add_argument('--sha256', default=os.environ.get('CONNECTUM_360_MODEL_SHA256', ''))
    p.add_argument('--device', choices=['cpu', 'cuda'], default='cuda')
    p.add_argument('--head-device', choices=['cpu', 'cuda', 'mps'])
    p.add_argument('--port', type=int, default=8093)
    p.add_argument('--direct-camera', action='store_true')
    p.add_argument('--video-device', default='/dev/video91')
    args = p.parse_args()
    if args.direct_camera:
        if args.device != 'cuda' or (args.head_device or args.device) != 'cuda':
            p.error('hardware direct camera requires CUDA encoder and heads')
        from surround_capture import HardwareCamera
        from surround_tensor import SurroundTensorProjector
        HardwareCamera.check_runtime()  # Plugins/CUDA only; do not open the camera before a task.
        projector = SurroundTensorProjector('flyvis')
    model = Connectome360Runtime(args.model, args.sha256, encoder_device=args.device,
                                head_device=args.head_device or args.device)
    if args.direct_camera:
        import torch
        model.infer_views_tensor(projector.project(
            torch.full((1440, 2880, 4), 127, dtype=torch.uint8, device='cuda')))
        model.details.update(camera_decode='nvv4l2decoder-mjpeg', camera_conversion='VIC-BGRx',
                             camera_preprocess='cuda', camera_memory='host-staging-to-cuda')
    else:
        model.infer(np.full((540, 960, 3), 127, np.uint8))
    model.reset()
    print(json.dumps(dict(ready=True, model=model.details)), flush=True)
    if args.direct_camera:
        from dataclasses import asdict
        from direct_camera_service import DirectCameraService, UsbSource, serve
        def predict(panorama):
            _, command, metrics = model.infer_views_tensor(projector.project(panorama))
            return dict(command=asdict(command), metrics=metrics)
        serve(DirectCameraService(model, UsbSource(args.video_device, camera_factory=HardwareCamera), predict), args.port)
        return
    make_server(Connectome360Service(model), '127.0.0.1', args.port,
                endpoint='/v1/360-road').serve_forever()


if __name__ == '__main__':
    main()

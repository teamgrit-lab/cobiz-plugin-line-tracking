#!/usr/bin/env python3
"""Camera-owned front Flyvis inference. No image transport or motion publisher."""
import argparse
import json

import numpy as np

from direct_camera_service import DirectCameraService, RosSource, learned_output, serve


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--model', required=True)
    p.add_argument('--port', type=int, default=8092)
    p.add_argument('--device', choices=['cpu', 'cuda'], default='cuda')
    p.add_argument('--head-device', choices=['cpu', 'mps', 'cuda'])
    p.add_argument('--camera-topic', required=True, help='Original A2 front camera ROS topic')
    args = p.parse_args()
    from front_connectome import FrontConnectome
    model = FrontConnectome(args.model, encoder_device=args.device, head_device=args.head_device or args.device)
    model.infer(np.zeros((270, 480, 3), np.uint8))
    model.reset()
    print(json.dumps(dict(ready=True, model=model.details)), flush=True)
    serve(DirectCameraService(model, RosSource(args.camera_topic),
                              lambda frame: learned_output(model, frame)), args.port)


if __name__ == '__main__':
    main()

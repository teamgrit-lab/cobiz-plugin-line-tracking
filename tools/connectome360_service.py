#!/usr/bin/env python3
"""Serialized four-view Flyvis inference. No camera access or motion publisher."""
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
    args = p.parse_args()
    model = Connectome360Runtime(args.model, args.sha256, encoder_device=args.device,
                                head_device=args.head_device or args.device)
    model.infer(np.full((540, 960, 3), 127, np.uint8))
    model.reset()
    print(json.dumps(dict(ready=True, model=model.details)), flush=True)
    make_server(Connectome360Service(model), '127.0.0.1', args.port,
                endpoint='/v1/360-road').serve_forever()


if __name__ == '__main__':
    main()

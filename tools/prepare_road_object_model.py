#!/usr/bin/env python3
"""Download the pinned official TorchVision detector; never operates the robot."""
import argparse
import hashlib
import json
from pathlib import Path
import torch

URL = 'https://download.pytorch.org/models/fasterrcnn_mobilenet_v3_large_320_fpn-907ea3f9.pth'

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path('models/road-objects-mobilenet-fpn.pth'))
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    if not args.output.exists():
        torch.hub.download_url_to_file(URL, str(args.output), hash_prefix='907ea3f9')
    digest = hashlib.sha256(args.output.read_bytes()).hexdigest()
    if not digest.startswith('907ea3f9'):
        raise ValueError('Existing detector file does not match official weights')
    metadata = {'model': 'torchvision/fasterrcnn_mobilenet_v3_large_320_fpn/COCO_V1', 'url': URL, 'sha256': digest}
    args.output.with_suffix('.json').write_text(json.dumps(metadata, indent=2)+'\n')
    print('LINE_TRACKING_ROAD_OBJECT_SHA256='+digest)

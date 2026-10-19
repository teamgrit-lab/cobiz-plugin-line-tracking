"""Export the checksum-verified official Flyvis 1.2.0 flow/0000/000 model.

Compatible with fly_connectom's flynav.encoder.v1 bundle. No download, training
or random initialization is performed by this exporter.
"""
import argparse
import hashlib
import io
import json
from pathlib import Path
import zipfile

import yaml

OFFICIAL_ARCHIVE_SHA256 = '71c78d4070556a536b13b23ee3139cd2788aa2a9d07d430a223b4edead281db1'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('archive', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    digest = hashlib.sha256()
    with args.archive.open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            digest.update(block)
    if digest.hexdigest() != OFFICIAL_ARCHIVE_SHA256:
        raise ValueError('official Flyvis archive checksum mismatch')
    import torch
    prefix = 'results/flow/0000/000/'
    with zipfile.ZipFile(args.archive) as archive:
        config = yaml.safe_load(archive.read(prefix+'_meta.yaml'))['config']
        raw = archive.read(prefix+'best_chkpt')
    checkpoint = torch.load(io.BytesIO(raw), map_location='cpu', weights_only=True)
    if config['task']['dataset']['tasks'] != ['flow']:
        raise ValueError('unexpected official training task')
    source = {'repository': 'https://github.com/TuragaLab/flyvis', 'release': 'v1.2.0',
              'archive_sha256': OFFICIAL_ARCHIVE_SHA256, 'member': prefix+'best_chkpt',
              'checkpoint_sha256': hashlib.sha256(raw).hexdigest(), 'pretrained_task': 'optical_flow'}
    bundle = {'format': 'flynav.encoder.v1', 'pretrained_checkpoint': source['member'],
              'pretrained_task': 'optical_flow', 'source': source,
              'network_config': config['network'], 'extent': config['network']['connectome']['extent'],
              'state_dict': checkpoint['network'], 'flow_decoder_config': config['task']['decoder']['flow'],
              'flow_decoder_state_dict': checkpoint['decoder']['flow']}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    torch.save(bundle, args.output)
    manifest = {**source, 'bundle_sha256': hashlib.sha256(args.output.read_bytes()).hexdigest(),
                'semantic_classes': [], 'trained_waypoint_head': False}
    args.output.with_suffix('.json').write_text(json.dumps(manifest, indent=2)+'\n')
    print(json.dumps(manifest, indent=2))


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""Download only the pinned public RGB road model; no scene data is uploaded."""
from pathlib import Path
import argparse
import hashlib
from huggingface_hub import snapshot_download
from dangjin_perception import MODEL_ID,REVISION

EXPECTED_SHA256='ffe3494e1339abf7af09a13c914e72c3d2745e2f315eba1fd2b1dee15b7a73ed'


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=Path('.cache/dangjin-segformer-b0'))
    args=parser.parse_args()
    snapshot_download(MODEL_ID,revision=REVISION,local_dir=args.output,token=False,
                      allow_patterns=['config.json','preprocessor_config.json','pytorch_model.bin'])
    if hashlib.sha256((args.output/'pytorch_model.bin').read_bytes()).hexdigest()!=EXPECTED_SHA256:
        raise ValueError('Pinned road model weight checksum mismatch')
    print(f'Verified {MODEL_ID}@{REVISION} in {args.output}')

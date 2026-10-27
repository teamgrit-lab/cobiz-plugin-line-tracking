#!/usr/bin/env python3
"""Bundle trained front weights, architecture, runtime and reproducibility files."""
import argparse
import ast
import hashlib
import importlib.metadata
import json
from pathlib import Path
import platform
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--model', type=Path, required=True)
    p.add_argument('--cache', type=Path, default=ROOT / '.cache/front-road-v1')
    p.add_argument('--exclusions', type=Path)
    args = p.parse_args(); dest = args.model
    config = json.loads((dest / 'model.json').read_text())
    for name, sha in config['weights_sha256'].items():
        assert hashlib.sha256((dest / name).read_bytes()).hexdigest() == sha
    runtime = dest / 'runtime'; runtime.mkdir(exist_ok=True)
    todo = ['front_connectome', 'front_connectome_service', 'front_connectome_client']; seen = set()
    while todo:
        name = todo.pop(); source = ROOT / 'tools' / (name + '.py')
        if name in seen or not source.exists():
            continue
        seen.add(name); shutil.copy2(source, runtime / source.name)
        for node in ast.walk(ast.parse(source.read_text())):
            if isinstance(node, ast.ImportFrom) and node.module:
                todo.append(node.module.split('.')[0])
            elif isinstance(node, ast.Import):
                todo.extend(a.name.split('.')[0] for a in node.names)
    source = dest / 'source'; source.mkdir(exist_ok=True)
    for pattern in ['tools/*front_connectome*.py', 'tools/validate_front_http.py', 'tools/audit_front_holdout.py', 'tools/filter_front_road_data.py', 'tools/train_connectome_policy.py',
                    'tools/dangjin_sensor_faults.py', 'tools/swin_l_local_path_debug.py',
                    'tools/validate_front_drive.mjs', 'tools/watch_front_validation.py', 'test/test_front_connectome.py',
                    'environments/dangjin/*.js', 'environments/dangjin/*.json',
                    'environments/dangjin/*.html', 'environments/dangjin/*.css',
                    'docker-compose.front-connectome.yml', 'Dockerfile.connectome',
                    'requirements-connectome.txt']:
        for file in ROOT.glob(pattern):
            target = source / file.relative_to(ROOT); target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(file, target)
    shutil.copy2(args.cache / 'entries.json', dest / 'training-samples.json')
    if args.exclusions:
        selection = dest / 'data-selection'; selection.mkdir(exist_ok=True)
        for file in args.exclusions.parent.iterdir():
            if file.is_file(): shutil.copy2(file, selection / file.name)
    # Preserve the exact initial weights needed to reproduce fine-tuning.
    shutil.copy2(ROOT / '.cache/dangjin-connectome-road/road-readout.pt', dest / 'initial-road-readout.pt')
    versions = {}
    for name in ['torch', 'torchvision', 'flyvis', 'numpy', 'scipy', 'opencv-python', 'datamate']:
        try:
            versions[name] = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            pass
    provenance = {'platform': platform.platform(), 'architecture': platform.machine(),
        'python': platform.python_version(), 'versions': versions,
        'repository_head': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
        'repository_dirty': True, 'source_snapshot': 'source/', 'runtime_modules': sorted(seen),
        'exclusions_sha256': hashlib.sha256(args.exclusions.read_bytes()).hexdigest() if args.exclusions else None,
        'preprocessing': {'width': 480, 'height': 270, 'resize': 'INTER_AREA', 'input': 'BGR uint8',
            'circuit': 'grayscale BoxEye, 10 Euler steps of 0.02 s from grey baseline per image',
            'road_features': [34, 48, 88], 'actor_mask_crop_y': [135, 257], 'actor_input': [1, 40, 96],
            'temporal_image_history': 1},
        'training': {'seed': 8427, 'actor_augmentation_seed': 7314, 'road_epochs': 24, 'actor_epochs': 100,
            'road_batch': 16, 'actor_batch': 48, 'road_lr': 0.00025, 'actor_lr': 0.001,
            'optimizer': 'AdamW + cosine decay', 'encoder_frozen': True,
            'real_control_weight': 0.35, 'simulation_control_weight': 1.,
            'real_targets': 'unverified pseudo labels; executed commands absent',
            'simulation_targets': 'offline geometry teacher; never provided at runtime'},
        'portability': 'CPU state_dict plus Python architecture; CUDA/TensorRT engines are not included.'}
    (dest / 'provenance.json').write_text(json.dumps(provenance, indent=2) + '\n')
    (dest / 'infer.py').write_text('''#!/usr/bin/env python3
from pathlib import Path
import sys, argparse, json
from dataclasses import asdict
sys.path.insert(0, str(Path(__file__).resolve().parent / "runtime"))
import cv2
from front_connectome import FrontConnectome
p = argparse.ArgumentParser(); p.add_argument("image"); p.add_argument("--device", default="cpu", choices=["cpu", "cuda"])
p.add_argument("--head-device", choices=["cpu", "mps", "cuda"]); p.add_argument("--mask")
a = p.parse_args(); frame = cv2.imread(a.image)
if frame is None: raise ValueError("Cannot read input image")
model = FrontConnectome(Path(__file__).resolve().parent, encoder_device=a.device, head_device=a.head_device or a.device)
mask, command, metrics = model.infer(frame)
if a.mask: cv2.imwrite(a.mask, mask * 255)
print(json.dumps({"command": asdict(command), "metrics": metrics, "model": model.details}, indent=2))
''')
    (dest / 'serve.py').write_text('''#!/usr/bin/env python3
from pathlib import Path
import sys, runpy
root = Path(__file__).resolve().parent
sys.path.insert(0, str(root / "runtime"))
if "--model" not in sys.argv: sys.argv += ["--model", str(root)]
runpy.run_module("front_connectome_service", run_name="__main__")
''')
    (dest / 'jetson.env.example').write_text('CONNECTUM_FRONT_MODEL_DIR=/home/USER/dev/danjin-a2/models/' + dest.name + '\n'
        + 'CONNECTUM_FRONT_SHA256=' + config['weights_sha256']['front_model.pt'] + '\n')
    readme = ROOT / 'docs/front-connectome.md'
    if readme.exists():
        shutil.copy2(readme, dest / 'README.md')
    manifest = {str(f.relative_to(dest)): hashlib.sha256(f.read_bytes()).hexdigest()
                for f in sorted(dest.rglob('*')) if f.is_file() and f.name != 'SHA256SUMS.json' and '__pycache__' not in f.parts}
    (dest / 'SHA256SUMS.json').write_text(json.dumps(manifest, indent=2) + '\n')
    print(json.dumps({'directory': str(dest), 'files': len(manifest), 'weights_sha256': config['weights_sha256']}))


if __name__ == '__main__':
    main()

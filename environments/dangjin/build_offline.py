#!/usr/bin/env python3
"""Pack the viewer and supplied local evidence into one browser-openable HTML.

Usage: python3 build_offline.py --bundle /path/to/esbuild-output.js
esbuild's only job is bundling the vendored Three.js modules. No files are sent
to a service. The generated HTML needs neither a server nor Internet access.
"""
import argparse
import base64
import json
import mimetypes
from pathlib import Path
import re

root = Path(__file__).resolve().parent


def build(bundle: Path, output: Path):
    assets, data = {}, {}
    evidence_roots=[root / 'evidence',root / 'evidence-september']
    for path in [root / "site.json", *(p for folder in evidence_roots for p in folder.glob('*.json'))]:
        data["./" + path.relative_to(root).as_posix()] = json.loads(path.read_text())
    for path in [*(p for folder in evidence_roots for p in folder.glob('camera-*.jpg')), *(root / "references").iterdir()]:
        mime = mimetypes.guess_type(path.name)[0]
        assets["./" + path.relative_to(root).as_posix()] = f"data:{mime};base64," + base64.b64encode(path.read_bytes()).decode()
    html = (root / "index.html").read_text()
    html = re.sub(r'\s*<script type="importmap">.*?</script>', '', html)
    html = html.replace('<link rel="stylesheet" href="./style.css">', '<style>' + (root / "style.css").read_text() + '</style>')
    # References remain inspectable without top-level data-URL navigation.
    for path in ("./references/site-plan.png", "./references/survey.jpg"):
        html = html.replace(f'href="{path}" target="_blank" rel="noopener"', f'href="{assets[path]}" download="{Path(path).name}"')
    for path, encoded in assets.items():
        html = html.replace(f'src="{path}"', f'src="{encoded}"')
    state = json.dumps({"assets": assets, "json": data}, ensure_ascii=False, separators=(",", ":")).replace('<', '\\u003c')
    js = bundle.read_text().replace('</script', '<\\/script')
    script = f'<script>window.DANGJIN_OFFLINE={state};</script>\n<script type="module">{js}</script>'
    html = html.replace('<script type="module" src="./app.js"></script>', script)
    html = html.replace('href="./" aria-label="당진 우사 처음으로"', 'href="#" aria-label="당진 우사 처음으로"')
    output.write_text(html)
    print(f"Created {output} ({output.stat().st_size / 1024 / 1024:.1f} MiB)")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bundle', type=Path, required=True)
    parser.add_argument('--output', type=Path, default=root / 'dangjin-environment.html')
    args = parser.parse_args()
    build(args.bundle, args.output)

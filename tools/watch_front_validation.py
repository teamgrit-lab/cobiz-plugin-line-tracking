#!/usr/bin/env python3
"""Read simulator telemetry and preserve a run before the next UI reset."""
import argparse
import json
from pathlib import Path
import time
import urllib.request

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--output',type=Path,required=True)
p.add_argument('--sha256',required=True)
p.add_argument('--max-wall-sec',type=float,default=900)
a=p.parse_args();start=time.monotonic();last_print=0.;run=None
while time.monotonic()-start<a.max_wall_sec:
    with urllib.request.urlopen('http://127.0.0.1:8773/api/telemetry',timeout=5) as response:run=json.load(response)
    assert run['policy']['weights_sha256']==a.sha256,'wrong model under test'
    if run['latest']:
        complete=run['mission'] and run['mission']['complete']
        violations=run['latest']['evaluation'].get('violation_frames',0)
        if complete or violations or run['errors']:
            a.output.write_text(json.dumps(run)+'\n')
            print(json.dumps({'saved':str(a.output),'complete':bool(complete),'violations':violations,'steps':run['steps']}),flush=True)
            raise SystemExit(0 if complete and not violations and not run['errors'] else 1)
        if time.monotonic()-last_print>30:
            print(json.dumps({'steps':run['steps'],'progress':run['mission']['fraction'] if run['mission'] else None,
                'distance_m':run['distance_m'],'stops':run['stops'],'violations':violations}),flush=True)
            last_print=time.monotonic()
    time.sleep(3)
if run:a.output.write_text(json.dumps(run)+'\n')
raise SystemExit('run did not complete within the wall-time bound')

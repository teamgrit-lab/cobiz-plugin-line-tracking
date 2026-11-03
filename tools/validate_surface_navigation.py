"""Replay identical cached masks through frozen/current controllers, without ROS.

Manifest datasets contain cache/{frames,masks}, source timestamps and configs.
No measured odometry is fabricated. Run separately with --source for each Git
snapshot so Python modules and state cannot leak across the before/after runs.
"""
import argparse
from collections import Counter
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import sys

import cv2
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    sys.path.insert(0, str(args.source.resolve()/'tools'))
    import local_path as lp
    from path_execution import PathExecution, PathExecutionConfig, make_path_tracker
    from swin_l_drive_control import (DriveConfig, DriveDecision, RoadCenterState,
                                      PathLossRecovery, decide_drive)
    has_corners = (args.source/'tools/road_corner.py').exists()
    if has_corners:
        from road_corner import CornerNavigator, CornerConfig
    manifest = json.loads(args.manifest.read_text())
    settings = dict(manifest['local_config'])
    for name in ('roi_polygon', 'road_roi_polygon'):
        settings[name] = tuple(settings[name])
    config = lp.LocalPathConfig(**settings)
    dc = DriveConfig(**manifest['drive_config'])
    execution = PathExecutionConfig(**manifest['execution_config'])
    cv2.setNumThreads(2)
    args.output.mkdir(parents=True, exist_ok=True)
    results = []
    for dataset in manifest['datasets']:
        times = dataset['times']
        cache = Path(dataset['cache'])
        trackers = {mode: make_path_tracker(config, execution, mode) for mode in (0, 1)}
        states = {mode: RoadCenterState() for mode in trackers}
        searches = {mode: PathLossRecovery(dc, output_hz=10.) for mode in trackers}
        corners = {mode: CornerNavigator(CornerConfig(**manifest['corner_config'])) if has_corners else None
                   for mode in trackers}
        last = {mode: (None, None) for mode in trackers}
        lost = {mode: 0 for mode in trackers}
        analyses, paths, commands = {}, {}, {}
        rows = []
        last_t = 0.

        def tick(t, inference_id, captured):
            for mode, tracker in trackers.items():
                if isinstance(tracker, PathExecution):
                    tracker.advance(t)
                path = tracker.current(t)
                analysis = analyses[mode]
                age = max(0., t-captured)
                # Match live drive_readiness: only actual semantic loss lifts
                # an execution guard into the independent loss-recovery FSM.
                execution_loss = isinstance(tracker, PathExecution) and tracker.path_loss_at is not None
                if (dc.path_loss_recovery_enabled and lost[mode] > 0
                        and not (analysis and analysis.blocked)
                        and (path is None or path.stop_reason is None or execution_loss)):
                    path = None
                cmd = decide_drive(path, camera_age_sec=age, inference_age_sec=age, config=dc,
                    road_state=states[mode], now=t, last_valid_forward_mps=last[mode][0],
                    last_valid_yaw_rate=last[mode][1], path_recovery_inferences=lost[mode],
                    road_clearance_blocked=bool(analysis and analysis.blocked),
                    road_path_unconfirmed=bool(mode == 1 or has_corners) and path is None)
                navigator = corners[mode]
                if navigator:
                    cmd = navigator.decide(cmd, path, now=t, pose=None, drive_config=dc,
                                           sensors_ready=age <= navigator.config.max_age_sec)
                if not cmd.reason.startswith('corner_') and not (navigator and navigator.active):
                    recovery = tracker.recovery_candidate(t) if isinstance(tracker, PathExecution) else None
                    cmd = searches[mode].decide(cmd, now=t, inference_id=inference_id,
                        inference_source_at=captured, sensors_ready=age <= dc.max_camera_age_sec,
                        sensor_reason='camera_stale', path_confidence=path.confidence if path else None,
                        recovery_candidate_confidence=recovery.confidence if recovery else None)
                states[mode].record_command(cmd, now=t)
                searches[mode].record_command(cmd, now=t)
                if navigator:
                    navigator.record_command(cmd, t)
                if cmd.reason in ('tracking', 'tracking_slow_turn'):
                    last[mode] = (cmd.vx, cmd.yaw_rate)
                if (not 0 <= cmd.vx <= dc.max_forward_mps+1e-9 or abs(cmd.vy) > .25+1e-9
                        or abs(cmd.yaw_rate) > dc.max_yaw_rps+1e-9):
                    raise AssertionError(('speed limit', dataset['id'], mode, t, cmd))
                paths[mode], commands[mode] = path, cmd

        for index, captured in enumerate(times):
            if index:
                for t in np.arange(last_t+.1, captured-1e-7, .1):
                    tick(float(t), index, last_t)
            with np.load(cache/'masks'/f'mask-{index:05}.npz') as data:
                selected = data['selected']
                hazard = data['hazard'] if 'hazard' in data else None
            item = {'index': index, 'time_sec': captured, 'modes': {}}
            for mode, tracker in trackers.items():
                region = lp.selected_path_region(selected, mode)
                if mode == 1 or has_corners:
                    surface_config = config.for_mask_class(mode) if hasattr(config, 'for_mask_class') else config
                    analysis = lp.analyze_road_path(region, surface_config, hazard)
                    if isinstance(tracker, PathExecution):
                        tracker.update_road(analysis, captured, observed_at=captured, frame_key=index+1)
                    else:
                        tracker.update_road(analysis, captured)
                    analyses[mode] = analysis
                    missing = (tracker.path_loss_at == captured if isinstance(tracker, PathExecution)
                               else tracker.current(captured) is None)
                    if corners[mode]:
                        corners[mode].observe(analysis, captured, observed_at=captured, frame_key=index+1)
                else:
                    estimate = lp.extract_sidewalk_centerline(region, config)
                    if estimate is None:
                        tracker.reset(preserve_branch=True)
                    tracker.update(estimate, captured)
                    analyses[mode] = None
                    missing = estimate is None
                lost[mode] = lost[mode]+1 if missing else 0
            tick(captured, index+1, captured)
            for mode in trackers:
                p, a, cmd = paths[mode], analyses[mode], commands[mode]
                valid = bool(p is not None and p.stop_reason is None)
                verified = None
                if valid and a is not None:
                    verified = a.corridor_for(p.points_xy) is not None
                    if not verified and has_corners:
                        raise AssertionError(('fresh mask', dataset['id'], index, mode))
                item['modes'][str(mode)] = {
                    'valid': valid, 'points': p.points_xy.tolist() if valid else None,
                    'geometry': p.road_geometry if valid else None,
                    'command': asdict(cmd), 'mask_verified': verified,
                    'status': a.status if a is not None else ('valid' if valid else 'missing'),
                    'corner': corners[mode].metrics() if corners[mode] else None,
                    'execution': trackers[mode].metrics(captured) if isinstance(trackers[mode], PathExecution) else None}
            rows.append(item)
            last_t = captured
            if index % 500 == 0:
                print(dataset['id'], index, '/', len(times), flush=True)
        dest = args.output/dataset['id']
        dest.mkdir(exist_ok=True)
        (dest/'traces.json').write_text(json.dumps(rows, ensure_ascii=False))
        summary = {'id': dataset['id'], 'samples': len(rows), 'duration_sec': dataset['duration_sec'], 'modes': {}}
        for mode in ('0', '1'):
            frames = [r['modes'][mode] for r in rows]
            summary['modes'][mode] = {
                'valid_paths': sum(f['valid'] for f in frames),
                'forward_frames': sum(f['command']['vx'] > 0 for f in frames),
                'slowed_frames': sum(0 < f['command']['vx'] < dc.max_forward_mps-.001 for f in frames),
                'yaw_frames': sum(abs(f['command']['yaw_rate']) > .001 for f in frames),
                'corner_candidates': sum(bool(f['corner'] and f['corner']['candidate']) for f in frames),
                'reasons': dict(Counter(f['command']['reason'] for f in frames))}
        results.append(summary)
        print(summary, flush=True)
    files = ['local_path.py', 'path_execution.py', 'swin_l_drive_control.py', 'road_curve.py',
             'swin_l_local_path_debug.py'] + (['road_corner.py'] if has_corners else [])
    (args.output/'summary.json').write_text(json.dumps({'results': results,
        'source': str(args.source.resolve()), 'odom_available': False,
        'selection_roi': {str(mode): list((config.for_mask_class(mode) if hasattr(config, 'for_mask_class')
                                         else config).road_roi_polygon
                                        if mode == 1 or has_corners else config.roi_polygon)
                          for mode in (0, 1)},
        'code_sha256': {f: hashlib.sha256((args.source/'tools'/f).read_bytes()).hexdigest() for f in files}},
        ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()

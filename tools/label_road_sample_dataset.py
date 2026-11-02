#!/usr/bin/env python3
"""Offline, explicitly provisional visual/LiDAR teacher labels; no robot I/O.

The source has no action ground truth. Pixel paths are reviewed proposals, the
extrinsic profile is unverified for these recordings, and speeds are assumptions.
"""
import argparse
import json
from collections import Counter
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw
from scipy.spatial import cKDTree

POLICY = {
    'version': 'provisional-geometric-teacher-v1',
    'straight_speed_mps': .25, 'bend_speed_mps': .18,
    'max_abs_yaw_rate_radps': .18, 'action_horizon_s': .5,
    'turn_heading_threshold_deg': 5.,
    'assumed_robot_half_width_with_margin_m': .65,
    'near_obstacle_hold_distance_m': 1.5,
    'ground_inlier_tolerance_m': .06, 'minimum_ground_coverage': .6,
    'obstacle_min_height_m': .18, 'obstacle_max_height_m': 2.5,
    'origin': 'Design assumptions for this offline pilot; not recorded commands, robot limits, or optimized/expert ground truth.',
    'steering': 'omega = v * 2*y/(x*x+y*y); base x forward, y left; no lateral velocity',
    'target': 'third annotated pixel waypoint intersected with a LiDAR-fitted local ground plane',
    'not_a_collision_checker': True,
}


def write_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n')


def ground_plane(points, seed=8123):
    """z = ax + by + c. Return a fit only when near-road evidence supports it."""
    candidate = points[(points[:, 0] > .7) & (points[:, 0] < 10) &
                       (abs(points[:, 1]) < 3) & (points[:, 2] < 0) & (points[:, 2] > -2)]
    if len(candidate) < 150:
        return None, {'valid': False, 'reason': 'too_few_ground_candidates'}
    rng = np.random.default_rng(seed)
    fit = candidate[rng.choice(len(candidate), min(4000, len(candidate)), replace=False)]
    A = np.column_stack([fit[:, :2], np.ones(len(fit))])
    best = np.zeros(len(fit), dtype=bool)
    for _ in range(180):
        index = rng.choice(len(fit), 3, replace=False)
        if abs(np.linalg.det(A[index])) < .01:
            continue
        c = np.linalg.solve(A[index], fit[index, 2])
        if np.linalg.norm(c[:2]) > .25 or not (-.9 < c[2] < -.2):
            continue
        inliers = abs(fit[:, 2] - A @ c) < POLICY['ground_inlier_tolerance_m']
        if inliers.sum() > best.sum():
            best = inliers
    if best.sum() < 150:
        return None, {'valid': False, 'reason': 'ground_ransac_failed'}
    c = np.linalg.lstsq(A[best], fit[best, 2], rcond=None)[0]
    residual = abs(fit[:, 2] - A @ c)
    inside = residual < POLICY['ground_inlier_tolerance_m']
    ratio = float(inside.mean())
    valid = ratio >= .55 and np.linalg.norm(c[:2]) <= .25 and -.9 < c[2] < -.2
    return (c if valid else None), {
        'valid': bool(valid), 'is_estimated': True, 'equation': 'z = a*x + b*y + c in assumed base frame',
        'coefficients': c.tolist(), 'candidate_points': len(candidate),
        'fit_points': len(fit), 'inlier_fraction': ratio,
        'median_inlier_residual_m': float(np.median(residual[inside])),
        'extrinsic_verified': False,
    }


def unproject_ground(pixels, K, D, camera_from_base, plane):
    if plane is None:
        return [None for _ in pixels]
    normalized = cv2.undistortPoints(np.asarray(pixels, np.float64).reshape(-1, 1, 2), K, D).reshape(-1, 2)
    base_from_camera = np.linalg.inv(camera_from_base)
    origin = base_from_camera[:3, 3]
    rays = np.column_stack([normalized, np.ones(len(normalized))]) @ base_from_camera[:3, :3].T
    normal = np.array([-plane[0], -plane[1], 1.])
    numerator = plane[2] - normal @ origin
    results = []
    for ray in rays:
        denominator = normal @ ray
        distance = numerator / denominator if abs(denominator) > 1e-9 else -1
        p = origin + distance * ray
        results.append(p.tolist() if distance > 0 and .4 < p[0] < 30 and abs(p[1]) < 15 else None)
    return results


def project_points(points, K, D, camera_from_base):
    camera = points @ camera_from_base[:3, :3].T + camera_from_base[:3, 3]
    uv = np.full((len(points), 2), np.nan)
    eligible = (camera[:, 2] > .3) & (np.linalg.norm(camera[:, :2], axis=1) / np.maximum(camera[:, 2], .01) < 1.5)
    if eligible.any():
        uv[eligible] = cv2.projectPoints(camera[eligible], np.zeros(3), np.zeros(3), K, D)[0].reshape(-1, 2)
    inside = eligible & np.isfinite(uv).all(1) & (uv[:, 0] >= 0) & (uv[:, 0] < 1280) & (uv[:, 1] >= 0) & (uv[:, 1] < 720)
    return uv, inside


def interpolate_annotations(keyframes, frame, episode_rows, field):
    by_index = {r['camera']['index']: int(r['camera']['header_time_ns']) for r in episode_rows}
    index = frame['camera']['index']
    before = max((k for k in keyframes if k['index'] <= index), key=lambda k: k['index'])
    after = min((k for k in keyframes if k['index'] >= index), key=lambda k: k['index'])
    t0, t1 = by_index[before['index']], by_index[after['index']]
    alpha = 0. if t0 == t1 else (int(frame['camera']['header_time_ns']) - t0) / (t1 - t0)
    values = (1-alpha) * np.array(before[field], float) + alpha * np.array(after[field], float)
    return values.tolist(), {'keyframe_indices': [before['index'], after['index']], 'weight_after': alpha,
                            'method': 'visual_keyframe' if t0 == t1 else 'time_interpolation_between_visual_keyframes',
                            'offline_context_used': True, 'human_verified': False}


def corridor_evidence(points, plane, target):
    if plane is None or target is None:
        return {'valid': False, 'reason': 'missing_ground_or_target'}, np.zeros(len(points), bool), np.zeros(len(points), bool), np.empty((0, 2))
    height = points[:, 2] - (points[:, :2] @ plane[:2] + plane[2])
    ground = abs(height) <= .09
    obstacles = (height >= POLICY['obstacle_min_height_m']) & (height <= POLICY['obstacle_max_height_m']) & (points[:, 0] > .7)
    x, y = target[:2]
    kappa = 2*y / (x*x + y*y)
    distance = min(8., max(2., np.hypot(x, y)))
    s = np.arange(.7, distance + .05, .1)
    path = np.column_stack([s, np.zeros(len(s))]) if abs(kappa) < 1e-6 else np.column_stack([np.sin(kappa*s)/kappa, (1-np.cos(kappa*s))/kappa])
    d, nearest = cKDTree(path).query(points[:, :2])
    corridor = d < POLICY['assumed_robot_half_width_with_margin_m']
    supported = ground & corridor
    bins = max(1, int(np.ceil((distance-.7)/.5)))
    support_counts = np.histogram(s[nearest[supported]], bins=bins, range=(.7, distance+.001))[0]
    coverage = float((support_counts >= 8).mean())
    # Require nearby non-ground neighbors rather than trusting isolated returns.
    robust = np.zeros(len(points), bool)
    candidate_ids = np.flatnonzero(obstacles & (points[:, 0] < 30) & (abs(points[:, 1]) < 15))
    if len(candidate_ids):
        counts = cKDTree(points[candidate_ids]).query_ball_point(points[candidate_ids], .22, return_length=True)
        robust[candidate_ids[counts >= 4]] = True
    near = robust & corridor
    closest = float(np.min(s[nearest[near]])) if near.any() else None
    return {'valid': True, 'is_estimated': True, 'path_curvature_per_m': float(kappa),
            'lookahead_path_length_m': distance, 'ground_points_in_corridor': int(supported.sum()),
            'ground_support_fraction': coverage, 'ground_support_bins': support_counts.tolist(),
            'non_ground_points_in_corridor': int(near.sum()), 'nearest_supported_obstacle_along_path_m': closest,
            'unknown_is_not_free': True, 'extrinsic_verified': False,
            'limitations': 'Local plane, incomplete scan coverage, no deskew, no swept-body or dynamics validation.'}, ground, robust, path


def propose_command(episode, target, evidence, association_valid, explicit_stop):
    reason = explicit_stop['reason'] if explicit_stop else None
    if not association_valid:
        reason = 'unreliable_camera_lidar_association'
    elif not evidence.get('valid'):
        reason = 'insufficient_geometry'
    elif evidence['ground_support_fraction'] < POLICY['minimum_ground_coverage']:
        reason = 'insufficient_ground_support'
    elif evidence['nearest_supported_obstacle_along_path_m'] is not None and evidence['nearest_supported_obstacle_along_path_m'] < POLICY['near_obstacle_hold_distance_m']:
        reason = 'near_non_ground_returns_in_proposed_corridor'
    if reason:
        v, omega, action = 0., 0., 'stop'
    else:
        v = POLICY['bend_speed_mps'] if episode == 'right_turn' else POLICY['straight_speed_mps']
        x, y = target[:2]
        omega = float(np.clip(v*2*y/(x*x+y*y), -POLICY['max_abs_yaw_rate_radps'], POLICY['max_abs_yaw_rate_radps']))
        heading = float(np.degrees(np.arctan2(y, x)))
        threshold = POLICY['turn_heading_threshold_deg']
        action = 'turn_right' if heading < -threshold else 'turn_left' if heading > threshold else 'forward'
    return {'action': action, 'linear_x_mps': v, 'linear_y_mps': 0., 'angular_z_radps': round(omega, 6),
            'stop': action == 'stop', 'stop_reason': reason, 'is_estimated': True,
            'label_status': 'provisional_teacher_requires_human_review',
            'confidence': 'low', 'confidence_is_calibrated': False,
            'confidence_basis': 'Visual path proposal, unverified extrinsics and assumed low speeds; no expert action ground truth.',
            'method': POLICY['version'], 'action_horizon_s': POLICY['action_horizon_s'],
            'actuation_delay_s': None, 'human_verified': False, 'deployable': False}


def render_review(root, row, points, ground, obstacles, path, uv, inside):
    im = Image.open(root / row['camera']['image']).convert('RGB')
    draw = ImageDraw.Draw(im)
    # Sparse projection diagnostics preserve the original image in images/.
    for index in np.flatnonzero(inside & (ground | obstacles))[::5]:
        x, y = uv[index]
        color = '#ff925b' if obstacles[index] else '#53dcca'
        draw.ellipse((x-1, y-1, x+1, y+1), fill=color)
    line = row['visual_annotation']['centerline_pixels']
    draw.line([tuple(v) for v in line], fill='#ffe27d', width=5)
    for x, y in line:
        draw.ellipse((x-5, y-5, x+5, y+5), fill='#ffe27d')
    tx, ty = line[2]
    draw.ellipse((tx-11, ty-11, tx+11, ty+11), outline='#ffffff', width=3)
    for obj in row['visual_annotation']['objects']:
        draw.rectangle(obj['bbox_xyxy'], outline='#fc8eaa', width=3)
        draw.text(tuple(obj['bbox_xyxy'][:2]), obj['track_id']+' (estimated box)', fill='#ffffff', stroke_width=2, stroke_fill='#171d28')
    draw.rectangle((0, 0, 1280, 36), fill='#101926')
    cmd = row['estimated_command']
    draw.text((14, 11), f"{row['id']} | ESTIMATE: {cmd['action']} v={cmd['linear_x_mps']:.2f} m/s, yaw={cmd['angular_z_radps']:+.3f} rad/s | recorded command: NONE", fill='white')
    relative = f"{row['episode']}/review/{row['camera']['index']:04d}-camera.jpg"
    im.resize((960,540)).save(root / relative, quality=89)
    row['review_camera'] = relative
    # BEV: x forward/up, y left/left, 15m forward and +/-8m lateral.
    bev = Image.new('RGB', (640, 640), '#101c29')
    d = ImageDraw.Draw(bev)
    def pixel(x, y): return (320-y*35, 590-x*35)
    for x in range(0, 16, 2):
        d.line([pixel(x,-8),pixel(x,8)], fill='#2a3b4b')
        d.text((8,pixel(x,0)[1]-8),f'{x} m',fill='#a5b7c4')
    for y in range(-8,9,2): d.line([pixel(0,y),pixel(16,y)],fill='#263747')
    for j in np.flatnonzero((points[:,0]>.2)&(points[:,0]<16)&(abs(points[:,1])<8))[::2]:
        color = '#f9905c' if obstacles[j] else '#4bbbad' if ground[j] else '#53616f'
        x,y = pixel(*points[j,:2]);d.ellipse((x-1,y-1,x+1,y+1),fill=color)
    if len(path):
        d.line([pixel(*p) for p in path], fill='#ffe27d', width=3)
    if row['geometry']['target_base_m'] is not None:
        x,y = pixel(*row['geometry']['target_base_m'][:2]);d.ellipse((x-6,y-6,x+6,y+6),outline='white',width=2)
    d.polygon([(310,590),(330,590),(320,570)],fill='white')
    d.rectangle((0,0,640,52),fill='#101c29')
    d.text((12,10),'LiDAR BEV | assumed extrinsics, local plane',fill='white')
    d.text((12,29),'cyan: ground | orange: non-ground | yellow: proposed path',fill='#b2c3ce')
    relative = f"{row['episode']}/review/{row['camera']['index']:04d}-bev.png"
    bev.save(root/relative);row['review_bev'] = relative


def build(root):
    raw = [json.loads(s) for s in (root/'raw_samples.jsonl').read_text().splitlines()]
    annotations = json.loads((root/'annotations.json').read_text())
    cal = json.loads((root/'calibration_assumptions.json').read_text())
    base_from_lidar = np.array(cal['base_from_lidar']);camera_from_base=np.array(cal['camera_from_base'])
    labels=[]
    for row in raw:
        ep = row['episode']; spec=annotations['episodes'][ep]
        episode_rows=[r for r in raw if r['episode']==ep]
        camera_info=json.loads((root/ep/'camera_info.json').read_text())
        K=np.array(camera_info['K']).reshape(3,3);D=np.array(camera_info['D'])
        pixels, provenance = interpolate_annotations(spec['centerline_keyframes'], row, episode_rows, 'pixels')
        cloud=np.load(root/row['lidar']['path']);sensor=cloud['xyz_sensor_m'][cloud['valid_point_mask']]
        points=sensor@base_from_lidar[:3,:3].T+base_from_lidar[:3,3]
        plane, fit=ground_plane(points)
        waypoints=unproject_ground(pixels,K,D,camera_from_base,plane);target=waypoints[2]
        evidence,ground,obstacles,path=corridor_evidence(points,plane,target)
        uv,inside=project_points(points,K,D,camera_from_base)
        objects=[]
        for obj in spec['objects']:
            index=row['camera']['index']
            if not obj['visible_first']<=index<=obj['visible_last']: continue
            box,bp=interpolate_annotations(obj['bbox_keyframes'],row,episode_rows,'xyxy')
            hit=inside&obstacles&(uv[:,0]>=box[0])&(uv[:,0]<=box[2])&(uv[:,1]>=box[1])&(uv[:,1]<=box[3])
            candidates=points[hit];ranges=np.linalg.norm(candidates[:,:2],axis=1)
            objects.append({'class':obj['class'],'track_id':obj['track_id'], 'bbox_xyxy':box,'is_estimated':True,
                'annotation_provenance':bp, 'identity_across_occlusion':'unknown',
                'projected_non_ground_points':int(hit.sum()),
                'candidate_range_p10_m':float(np.percentile(ranges,10)) if len(ranges)>=5 else None,
                'candidate_range_median_m':float(np.median(ranges)) if len(ranges)>=5 else None,
                'range_status':'unverified_projected_candidates_not_instance_depth',
                'range_caveat':'Foreground/background returns may project inside a coarse 2D box; object-specific distance is NOT verified.'})
        stop=next((s for s in spec['stop_intervals'] if s['first']<=row['camera']['index']<=s['last']),None)
        command=propose_command(ep,target,evidence,row['lidar']['association_valid'],stop)
        reason_ko = stop['reason_ko'] if stop else spec['reason_ko']
        if command['stop_reason'] == 'insufficient_ground_support':
            reason_ko = '단일 LiDAR 스캔에서 제안 경로의 지면 근거가 부족하여 판단 보류 정지를 제안합니다. 장애물 검출이나 전문가의 정지 정답이 아닙니다.'
        row.update(schema_version='1.0', estimated_command=command,
            visual_annotation={'is_estimated':True,'intent':spec['intent'],'centerline_pixels':pixels,'provenance':provenance,
                'confidence':spec['confidence'],'confidence_is_calibrated':False,'objects':objects,
                'reason_ko':reason_ko,
                'scope':'coarse path proposal, not a dense drivable mask'},
            geometry={'is_estimated':True,'ground_plane':fit,'waypoints_base_m':waypoints,'target_base_m':target,
                'corridor':evidence,'extrinsic_verified':False},
            observed_motion=None, observed_motion_status='not_estimated; IMU is raw evidence, GPS timebase is unreliable',
            training={'group_id':row['source_file'],'split':'review_only','approved_for_training':False,
                'requires_human_review':True,'human_verified':False,'camera_views':['front'],
                'no_random_frame_split':True,'teacher_uses_lidar':True,'teacher_uses_offline_visual_context':True,
                'command_ground_truth_available':False,
                'exclusion_reasons':['human_review_pending','unverified_extrinsics','actuation_delay_unknown'] +
                    (['insufficient_lidar_ground_support'] if command['stop_reason']=='insufficient_ground_support' else [])})
        render_review(root,row,points,ground,obstacles,path,uv,inside)
        labels.append(row)
    with (root/'labels.jsonl').open('w') as f:
        for row in labels: f.write(json.dumps(row,ensure_ascii=False,allow_nan=False)+'\n')
    write_json(root/'teacher_policy.json',POLICY)
    report={
        'samples':len(labels),'episodes':dict(Counter(r['episode'] for r in labels)),
        'estimated_actions':dict(Counter(r['estimated_command']['action'] for r in labels)),
        'recorded_commands':sum(r['recorded_command'] is not None for r in labels),
        'human_verified_labels':0,'training_approved_labels':0,
        'all_estimates_explicit':all(r['estimated_command']['is_estimated'] for r in labels),
        'invalid_camera_lidar_associations':sum(not r['lidar']['association_valid'] for r in labels),
        'camera_lidar_abs_midpoint_delta_ms_max':max(abs(r['lidar']['camera_delta_ms']) for r in labels),
        'invalid_ground_fits':sum(not r['geometry']['ground_plane']['valid'] for r in labels),
        'ground_support_min':min(r['geometry']['corridor'].get('ground_support_fraction',0) for r in labels),
        'stop_reasons':dict(Counter(r['estimated_command']['stop_reason'] for r in labels if r['estimated_command']['stop'])),
        'gps_diagnostic_samples':sum(r['gps_velocity'] is not None for r in labels),
        'gps_timebase_rejected_samples':sum(r['gps_velocity'] is not None and not r['gps_velocity']['usable_for_temporal_training'] for r in labels),
        'frame_gaps':[], 'limitations':['No command ground truth, front camera only, unverified extrinsics, no deskew.',
            'Offline visual proposals and policy assumptions; no claim of optimal or collision-free closed-loop behavior.']}
    for ep in annotations['episodes']:
        er=[r for r in labels if r['episode']==ep]
        for left,right in zip(er,er[1:]):
            dt=(int(right['camera']['header_time_ns'])-int(left['camera']['header_time_ns']))/1e9
            if dt>.8:report['frame_gaps'].append({'after_id':left['id'],'before_id':right['id'],'seconds':dt,'filled':False})
    write_json(root/'quality_report.json',report)
    write_json(root/'review_data.json',labels)
    # File:// usable viewer: the JSON remains separately available to training tools.
    (root/'review_data.js').write_text('window.ROAD_SAMPLES = '+json.dumps(labels,ensure_ascii=False,allow_nan=False)+';\n')
    print(json.dumps(report,ensure_ascii=False,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--dataset',type=Path,required=True)
    build(p.parse_args().dataset)

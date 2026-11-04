"""Estimate tag bearing and parallel heading from recorded 2:1 panoramas.

The CLI is offline; the detection helpers also serve the task-scoped live camera.
No ROS imports or motion publishers. Angles increase with panorama x; their signs
are not robot yaw commands. Alignment is not a planned trajectory.
"""
from __future__ import annotations

import argparse
from collections import Counter
import json
import math
from pathlib import Path
import sys

import numpy as np


def wrap_degrees(angle: float) -> float:
    return (angle + 180.0) % 360.0 - 180.0


def bearing_error(tag_x: float, axis_x: float, width: int) -> float:
    """Shortest horizontal error; positive is increasing panorama x."""
    if width <= 0 or not all(math.isfinite(v) for v in (tag_x, axis_x)):
        raise ValueError('finite positions and positive panorama width required')
    return wrap_degrees((tag_x - axis_x) * 360.0 / width)


def plane_error(normal_heading: float, forward_heading: float, parallel: bool) -> float:
    """Unsigned plane orientation has a 180-degree period, unlike a bearing."""
    tangent = normal_heading + (90.0 if parallel else 0.0)
    return (tangent - forward_heading + 90.0) % 180.0 - 90.0


def marker_center(corners: np.ndarray) -> np.ndarray:
    """Intersection of diagonals, preserving perspective rather than averaging."""
    points = np.column_stack((np.asarray(corners).reshape(4, 2), np.ones(4)))
    center = np.cross(np.cross(points[0], points[2]), np.cross(points[1], points[3]))
    if not np.isfinite(center).all() or abs(center[2]) < 1e-9:
        raise ValueError('degenerate marker quadrilateral')
    return center[:2] / center[2]


def estimate_plane(corners, view_size, view_x, width, forward_x, tolerance_deg,
                   tag_size_m=None, camera_offset_xy=(0., 0.), camera_z_m=0.,
                   camera_roll_pitch_rad=(0., 0.)):
    """Use a unit square: orientation needs no physical tag-size assumption.

    Both planar PnP solutions are checked. A small reprojection-error difference
    cannot resolve substantially different headings, so those remain ambiguous.
    """
    import cv2

    corners = np.asarray(corners, np.float64).reshape(4, 2)
    # The tag's printed rotation is irrelevant to its plane. Put the image's
    # upper-left corner first and use an image-down object y axis. This avoids
    # a near-pi Rodrigues singularity for an otherwise frontal upright tag.
    corners = np.roll(corners, -int(np.argmin(corners.sum(axis=1))), axis=0)
    edges = np.linalg.norm(corners - np.roll(corners, 1, axis=0), axis=1)
    if float(edges.min()) < 12.0:
        return dict(pose_status='tag_too_small')
    half = .5
    square = np.array([[-half, -half, 0], [half, -half, 0],
                       [half, half, 0], [-half, half, 0]], np.float64)
    focal, center = view_size / 2.0, (view_size - 1) / 2.0
    intrinsics = np.array([[focal, 0, center], [0, focal, center], [0, 0, 1.]])
    angle = (view_x - forward_x) * (2 * math.pi / width)
    c, s = math.cos(angle), math.sin(angle)
    roll, pitch = camera_roll_pitch_rad
    cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    def to_body(vector):
        vx, vy, vz = vector
        x, y, z = c * vz - s * vx, -s * vz - c * vx, -vy
        y, z = cr * y - sr * z, sr * y + cr * z
        x, z = cp * x + sp * z, -sp * x + cp * z
        return [float(x), float(y), float(z)]
    try:
        result = cv2.solvePnPGeneric(square, corners, intrinsics, None,
                                     flags=cv2.SOLVEPNP_IPPE)
        solutions = []
        for rvec, tvec in zip(result[1], result[2]):
            if not np.isfinite(rvec).all() or not np.isfinite(tvec).all():
                continue
            rotation, _ = cv2.Rodrigues(rvec)
            if np.any((square @ rotation.T + tvec.reshape(3))[:, 2] <= 0):
                continue
            projected, _ = cv2.projectPoints(square, rvec, tvec, intrinsics, None)
            error = float(np.sqrt(np.mean(np.sum(
                (projected.reshape(4, 2) - corners) ** 2, axis=1))))
            if not math.isfinite(error):
                continue
            normal = rotation[:, 2]
            nx, ny, _ = to_body(normal)
            if math.hypot(nx, ny) < .3:
                continue
            heading = wrap_degrees(forward_x * 360.0 / width - math.degrees(math.atan2(ny, nx)))
            solutions.append((error, heading, tvec.reshape(3), normal))
    except cv2.error:
        return dict(pose_status='pnp_failed')
    solutions.sort(key=lambda solution: solution[0])
    if not solutions or solutions[0][0] > 2.0:
        return dict(pose_status='unreliable_pose')
    error, heading, translation, normal = solutions[0]
    metric = {}
    if tag_size_m is not None:
        if not math.isfinite(tag_size_m) or tag_size_m <= 0:
            raise ValueError('positive finite tag size required')
        reliable_translation = (len(solutions) == 1 or solutions[1][0] - error > .25 or
            np.linalg.norm(translation - solutions[1][2]) <= .05 * np.linalg.norm(translation))
        if reliable_translation:
            position = to_body(translation * tag_size_m)
            position[0] += camera_offset_xy[0]
            position[1] += camera_offset_xy[1]
            position[2] += camera_z_m
            metric = dict(range_m=float(np.linalg.norm(translation) * tag_size_m),
                          position_body_m=position, normal_body=to_body(normal),
                          plane_distance_m=float(abs(normal @ translation) * tag_size_m))
    if len(solutions) > 1:
        other_error, other_heading, _, _ = solutions[1]
        disagreement = abs(plane_error(other_heading, heading, False))
        if other_error - error <= .25 and disagreement > tolerance_deg:
            return dict(pose_status='ambiguous_pose', reprojection_error_px=error, **metric)
    return dict(pose_status='ok', reprojection_error_px=error,
                parallel_error_deg=plane_error(heading, forward_x * 360 / width, True),
                perpendicular_error_deg=plane_error(heading, forward_x * 360 / width, False), **metric)


class PanoramaDetector:
    """Overlapping 90-degree perspective views, including the panorama seam."""

    def __init__(self, width, height, front_x, tolerance_deg=3., view_size=960,
                 family='36h11', tag_ids=None, tag_size_m=None, camera_offset_xy=(0., 0.),
                 camera_z_m=0., camera_roll_pitch_rad=(0., 0.)):
        import cv2

        if width != height * 2 or view_size < 128:
            raise ValueError('2:1 panorama and view size >= 128 required')
        if not hasattr(cv2, 'aruco'):
            raise RuntimeError('OpenCV with the aruco/AprilTag module required')
        self.width, self.height, self.front_x = width, height, front_x
        self.tolerance_deg, self.view_size, self.tag_ids = tolerance_deg, view_size, tag_ids
        self.tag_size_m, self.camera_offset_xy = tag_size_m, camera_offset_xy
        self.camera_z_m, self.camera_roll_pitch_rad = camera_z_m, camera_roll_pitch_rad
        dictionary = cv2.aruco.getPredefinedDictionary(
            getattr(cv2.aruco, 'DICT_APRILTAG_' + family))
        if tag_ids is not None and any(i >= len(dictionary.bytesList) for i in tag_ids):
            raise ValueError('tag_id_outside_family')
        params = (cv2.aruco.DetectorParameters() if hasattr(cv2.aruco, 'DetectorParameters')
                  else cv2.aruco.DetectorParameters_create())
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        params.errorCorrectionRate = 0.0
        if hasattr(cv2.aruco, 'ArucoDetector'):
            self.detect_markers = cv2.aruco.ArucoDetector(dictionary, params).detectMarkers
        else:
            # Ubuntu 22.04's system OpenCV, used by the Jetson image.
            self.detect_markers = lambda image: cv2.aruco.detectMarkers(image, dictionary, parameters=params)
        y, x = np.mgrid[:view_size, :view_size]
        focal, center = view_size / 2., (view_size - 1) / 2.
        horizontal = (x - center) / focal
        vertical = (y - center) / focal
        offset = np.arctan(horizontal) * width / (2 * np.pi)
        latitude = np.arctan2(-vertical, np.sqrt(1 + horizontal ** 2))
        map_y = ((.5 - latitude / np.pi) * height - .5).astype(np.float32)
        self.maps = [(view_x, ((view_x + offset) % width).astype(np.float32), map_y)
                     for view_x in ((front_x + i * width / 6.) % width for i in range(6))]

    def detect(self, panorama):
        import cv2

        if panorama.shape != (self.height, self.width, 3):
            raise ValueError('panorama dimensions changed')
        gray = cv2.cvtColor(panorama, cv2.COLOR_BGR2GRAY)
        found = []
        focal, center = self.view_size / 2., (self.view_size - 1) / 2.
        for view_x, map_x, map_y in self.maps:
            view = cv2.remap(gray, map_x, map_y, cv2.INTER_LINEAR,
                             borderMode=cv2.BORDER_WRAP)
            corners, ids, _ = self.detect_markers(view)
            if ids is None:
                continue
            for quad, tag_id in zip(corners, ids.flatten()):
                tag_id = int(tag_id)
                if self.tag_ids is not None and tag_id not in self.tag_ids:
                    continue
                try:
                    point = marker_center(quad)
                except ValueError:
                    continue
                horizontal, vertical = (point - center) / focal
                x = (view_x + math.atan(horizontal) * self.width / (2 * math.pi)) % self.width
                latitude = math.atan2(-vertical, math.sqrt(1 + horizontal ** 2))
                y = (.5 - latitude / math.pi) * self.height - .5
                item = dict(tag_id=tag_id, panorama_x=float(x), panorama_y=float(y),
                            off_axis=float(math.hypot(horizontal, vertical)),
                            corners=quad, view_x=view_x)
                # Multiple projections of one observation count as one tag.
                duplicate = next((index for index, old in enumerate(found) if old['tag_id'] == tag_id
                    and abs(bearing_error(x, old['panorama_x'], self.width)) < 2.
                    and abs(y - old['panorama_y']) * 180 / self.height < 2.), None)
                if duplicate is None:
                    found.append(item)
                elif item['off_axis'] < found[duplicate]['off_axis']:
                    found[duplicate] = item
        output = []
        for item in found:
            pose = estimate_plane(item.pop('corners'), self.view_size, item.pop('view_x'),
                                  self.width, self.front_x, self.tolerance_deg,
                                  self.tag_size_m, self.camera_offset_xy, self.camera_z_m,
                                  self.camera_roll_pitch_rad)
            item.pop('off_axis')
            output.append(dict(item, **pose))
        return output


class AlignmentMonitor:
    def __init__(self, width, front_x, rear_x, tolerance_deg=3., confirm_frames=3,
                 max_gap_sec=1.):
        if (width <= 0 or not all(math.isfinite(v) for v in
                (front_x, rear_x, tolerance_deg, max_gap_sec))
                or not 0 <= front_x < width or not 0 <= rear_x < width
                or front_x == rear_x or not 0 < tolerance_deg < 90
                or type(confirm_frames) is not int or confirm_frames < 1 or max_gap_sec <= 0):
            raise ValueError('invalid alignment axes, tolerance or confirmation limits')
        self.width, self.front_x, self.rear_x = width, front_x, rear_x
        self.tolerance, self.confirm_frames, self.max_gap = tolerance_deg, confirm_frames, max_gap_sec
        self.hits = {}
        self.last_stamp = None

    def observe(self, detections, stamp_sec):
        if not math.isfinite(stamp_sec) or stamp_sec < 0:
            raise ValueError('non-negative finite frame timestamp required')
        if self.last_stamp is not None and stamp_sec <= self.last_stamp:
            raise ValueError('frame timestamps must increase; repeated frames cannot confirm alignment')
        if self.last_stamp is not None and stamp_sec - self.last_stamp > self.max_gap:
            self.hits = {}
        self.last_stamp = stamp_sec
        counts = Counter(item['tag_id'] for item in detections)
        next_hits, output = {}, []
        for item in detections:
            front = bearing_error(item['panorama_x'], self.front_x, self.width)
            rear = bearing_error(item['panorama_x'], self.rear_x, self.width)
            axis = 'front' if abs(front) < abs(rear) else 'rear'
            if abs(abs(front) - abs(rear)) < 1e-8:
                axis = 'equidistant'
            unique = counts[item['tag_id']] == 1
            centered = unique and min(abs(front), abs(rear)) <= self.tolerance
            parallel = (unique and item.get('pose_status') == 'ok'
                        and abs(item['parallel_error_deg']) <= self.tolerance)
            old_axis, old_center, old_parallel = self.hits.get(item['tag_id'], (None, 0, 0))
            center_hits = (old_center + 1 if old_axis == axis else 1) if centered else 0
            parallel_hits = old_parallel + 1 if parallel else 0
            next_hits[item['tag_id']] = axis, center_hits, parallel_hits
            output.append(dict(item, front_error_deg=front, rear_error_deg=rear,
                nearest_axis=axis, center_confirm_frames=center_hits,
                horizontal_centered=center_hits >= self.confirm_frames,
                parallel_confirm_frames=parallel_hits,
                parallel_aligned=parallel_hits >= self.confirm_frames,
                observation_status='unique' if unique else 'multiple_same_id'))
        self.hits = next_hits  # No detection clears confirmation; nothing is extrapolated.
        return output


def annotate(frame, detections, front_x, rear_x, stamp_sec):
    import cv2

    preview = frame.copy()
    height, width = preview.shape[:2]
    header_height = min(height - 1, 115 + len(detections) * 45)
    cv2.rectangle(preview, (0, 0), (width - 1, header_height), (24, 20, 16), -1)
    for axis, x, color in [('FRONT', front_x, (50, 220, 255)),
                            ('REAR', rear_x, (230, 140, 255))]:
        cv2.line(preview, (round(x), 0), (round(x), height - 1), color, 3)
        cv2.putText(preview, f'{axis} x={x:.1f}', (max(0, min(width - 450, round(x) + 8)), 45),
                    cv2.FONT_HERSHEY_SIMPLEX, 1., color, 2)
    text = f'SOURCE {stamp_sec:.3f}s | ' + (
        'NO TAG' if not detections else 'HORIZONTAL BEARING / PARALLEL HEADING ESTIMATES')
    cv2.putText(preview, text, (20, 90), cv2.FONT_HERSHEY_SIMPLEX, .9, (255, 255, 255), 2)
    for index, tag in enumerate(detections):
        cv2.circle(preview, (round(tag['panorama_x']), round(tag['panorama_y'])), 12, (0, 255, 0), 3)
        parallel = tag.get('parallel_error_deg')
        pose_text = f'{parallel:+.2f} deg' if parallel is not None else tag['pose_status']
        line = (f"ID {tag['tag_id']} | front {tag['front_error_deg']:+.2f} | "
                f"rear {tag['rear_error_deg']:+.2f} | parallel {pose_text} | "
                f"parallel_aligned={tag['parallel_aligned']}")
        cv2.putText(preview, line, (20, 140 + index * 42), cv2.FONT_HERSHEY_SIMPLEX,
                    .85, (0, 255, 0), 2)
    return preview


def write_preview_sample(writer, frame, stamp_sec, start_sec, sample_hz, next_slot):
    """Keep source gaps visible instead of accelerating preview playback."""
    import cv2

    slot = math.floor((stamp_sec - start_sec) * sample_hz + 1e-8)
    while next_slot < slot:
        missing = np.zeros_like(frame)
        text = f'NO SAMPLED SOURCE FRAME | {start_sec + next_slot / sample_hz:.3f}s'
        cv2.putText(missing, text, (20, 90), cv2.FONT_HERSHEY_SIMPLEX,
                    .9, (200, 200, 255), 2)
        writer.write(missing)
        next_slot += 1
    writer.write(frame)
    return slot + 1


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--video', type=Path, required=True)
    parser.add_argument('--front-center-x', type=float, help='Default: 1/4 of source width (provisional)')
    parser.add_argument('--rear-center-x', type=float, help='Default: 3/4 of source width (provisional)')
    parser.add_argument('--tag-id', type=int, nargs='+', help='Default: report every detected ID')
    parser.add_argument('--family', choices=['16h5', '25h9', '36h10', '36h11'], default='36h11')
    parser.add_argument('--tolerance-deg', type=float, default=3.)
    parser.add_argument('--confirm-frames', type=int, default=3)
    parser.add_argument('--sample-hz', type=float, default=5.)
    parser.add_argument('--start-sec', type=float, default=0.)
    parser.add_argument('--duration-sec', type=float)
    parser.add_argument('--view-size', type=int, default=960)
    parser.add_argument('--output', type=Path, help='JSONL file; defaults to stdout')
    parser.add_argument('--preview', type=Path, help='Optional annotated MP4 at sample-hz')
    args = parser.parse_args(argv)
    if (not math.isfinite(args.sample_hz) or not 0 < args.sample_hz <= 30
            or not math.isfinite(args.start_sec) or args.start_sec < 0
            or (args.duration_sec is not None and
                (not math.isfinite(args.duration_sec) or args.duration_sec <= 0))):
        parser.error('sample-hz must be 0..30; start/duration must be finite and in range')
    outputs = [p.resolve() for p in (args.output, args.preview) if p is not None]
    if (args.video.resolve() in outputs or len(set(outputs)) != len(outputs)
            or any(p.exists() for p in outputs)):
        parser.error('output paths must be distinct, new files and must not replace the input')
    import cv2

    cap = cv2.VideoCapture(str(args.video))
    writer, handle = None, None
    try:
        if not cap.isOpened():
            parser.error(f'cannot open video: {args.video}')
        width, height = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        front = args.front_center_x if args.front_center_x is not None else width / 4.
        rear = args.rear_center_x if args.rear_center_x is not None else width * 3 / 4.
        try:
            monitor = AlignmentMonitor(width, front, rear, args.tolerance_deg, args.confirm_frames,
                                       max(1., 2. / args.sample_hz))
            detector = PanoramaDetector(width, height, front, args.tolerance_deg, args.view_size,
                                        args.family, set(args.tag_id) if args.tag_id else None)
        except (ValueError, RuntimeError) as error:
            parser.error(str(error))
        if args.start_sec and not cap.set(cv2.CAP_PROP_POS_MSEC, args.start_sec * 1000):
            parser.error('video does not support seeking to start-sec')
        if args.preview:
            writer = cv2.VideoWriter(str(args.preview), cv2.VideoWriter_fourcc(*'mp4v'),
                                     args.sample_hz, (width, height))
            if not writer.isOpened():
                parser.error('cannot create preview video')
        handle = args.output.open('x') if args.output else sys.stdout
        config = dict(type='configuration', schema='panorama.tag_alignment.v1',
            source=str(args.video.resolve()), width=width, height=height,
            front_center_x=front, rear_center_x=rear, axes_calibration='provisional',
            tolerance_deg=args.tolerance_deg, confirm_frames=args.confirm_frames,
            angle_sign='positive_towards_increasing_panorama_x',
            parallel_heading_reference='panorama_level_with_front_axis',
            physical_extrinsics_calibrated=False, family=args.family,
            scope='horizontal bearing and tag-plane heading; no motion commands')
        print(json.dumps(config, allow_nan=False), file=handle)
        next_sample, count, tagged, preview_slot = args.start_sec, 0, 0, 0
        while cap.grab():
            stamp = cap.get(cv2.CAP_PROP_POS_MSEC) / 1000.
            if not math.isfinite(stamp) or stamp < 0:
                raise ValueError('video did not provide a valid decoded-frame timestamp')
            if args.duration_sec is not None and stamp >= args.start_sec + args.duration_sec:
                break
            if stamp + 1e-8 < next_sample:
                continue
            ok, frame = cap.retrieve()
            if not ok:
                raise RuntimeError('failed to retrieve sampled video frame')
            detections = monitor.observe(detector.detect(frame), stamp)
            row = dict(type='frame', timestamp_sec=stamp,
                       frame_index=int(cap.get(cv2.CAP_PROP_POS_FRAMES)) - 1,
                       status='tags_detected' if detections else 'no_tag', detections=detections)
            print(json.dumps(row, allow_nan=False), file=handle)
            if writer:
                preview_slot = write_preview_sample(writer,
                    annotate(frame, detections, front, rear, stamp), stamp,
                    args.start_sec, args.sample_hz, preview_slot)
            count += 1
            tagged += bool(detections)
            next_sample = args.start_sec + (math.floor((stamp - args.start_sec) * args.sample_hz + 1e-8) + 1) / args.sample_hz
        if count == 0:
            raise ValueError('no frames sampled; check start-sec and duration-sec')
        print(f'sampled_frames={count} frames_with_tags={tagged}', file=sys.stderr)
    finally:
        cap.release()
        if writer is not None:
            writer.release()
        if handle is not None and handle is not sys.stdout:
            handle.close()


if __name__ == '__main__':
    main()

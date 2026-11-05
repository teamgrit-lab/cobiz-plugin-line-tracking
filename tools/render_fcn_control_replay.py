"""Replay the existing browser FCN Road controller on verified recorded masks.

This is open-loop recorded-video analysis, not a vehicle rollout. The source
camera follows its recorded motion independently of the generated commands.
The 1024 detector is adapted to the existing 8x16 controller contract explicitly.
"""
import argparse
from collections import Counter
from dataclasses import asdict
import csv
import hashlib
import json
import math
from pathlib import Path
import shutil
import subprocess
import time

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from fcn_replay import FcnDrive
from fcn360_road import (ASPECT, BLIND, EGO, FRONT_ROBOT_COLUMNS, FRONT_ROBOT_ROWS, FRONT_ROBOT_REVISION, ROAD,
                        apply_front_robot_prior, front_robot_mask, ground_sample, observed_classes)
from panorama_mask import panorama_mask
from validate_fcn_resolution_video import COLORS

MODELS = ("fcn512_finetuned", "fcn1024_finetuned_compat")
CODE = ("fcn_replay.py", "fcn360_road.py", "local_path.py", "drive_control.py")


def sha(path):
    with path.open("rb") as fp:
        return hashlib.file_digest(fp, "sha256").hexdigest()


def write_json(path, data):
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False) + "\n")


def legacy_grid(native):
    """Apply fixed robot labels, then pool; ambiguous nonroad ties stay unknown."""
    native = apply_front_robot_prior(native)
    if native.shape == (4, 8, 16):
        return native.copy()
    if native.shape != (4, 16, 32) or native.max() > 20:
        raise ValueError("Expected four native 16x32 semantic grids")
    blocks = native.reshape(4, 8, 2, 16, 2).transpose(0, 1, 3, 2, 4).reshape(4, 8, 16, 4)
    counts = np.stack([(blocks == i).sum(-1) for i in range(21)], axis=-1)
    is_road = counts[..., ROAD] >= 3
    counts[..., ROAD] = -1
    maximum = counts.max(-1)
    ambiguous = (counts == maximum[..., None]).sum(-1) != 1
    result = np.where(ambiguous, 0, counts.argmax(-1))
    return np.where(is_road, ROAD, result).astype(np.uint8)


def action(vx, yaw):
    if abs(vx) < 1e-8 and abs(yaw) < 1e-8:
        return "stop"
    if yaw > .03:
        return "left"
    if yaw < -.03:
        return "right"
    return "forward"


def command_arc(vx, yaw, horizon=4.0):
    t = np.linspace(0, horizon, 81)
    if abs(yaw) < 1e-8:
        return np.column_stack((vx * t, np.zeros_like(t)))
    return np.column_stack((vx * np.sin(yaw * t) / yaw,
                            vx * (1 - np.cos(yaw * t)) / yaw))


def replay(analysis, out, speed, height):
    if (out / "controller-trace.jsonl").exists():
        raise FileExistsError("Refusing to replace an existing controller replay")
    out.mkdir(parents=True, exist_ok=True)
    (out / "source").mkdir()
    (out / "previews").mkdir()
    source_complete = json.loads((analysis / "complete.json").read_text())
    for name in ("native-class-masks.npz", "frames.jsonl", "provenance.json"):
        if sha(analysis / name) != source_complete["sha256"][name]:
            raise ValueError(f"Changed input artifact: {name}")
    frames = [json.loads(x) for x in (analysis / "frames.jsonl").read_text().splitlines()]
    with np.load(analysis / "native-class-masks.npz") as archive:
        masks = {key: archive[key] for key in archive.files}
    assert np.array_equal(masks["pts"], [r["source_pts"] for r in frames])
    assert np.array_equal(masks["times"], [r["source_time_sec"] for r in frames])
    controllers = [FcnDrive(height, speed) for _ in MODELS]
    grids = {key: np.empty((len(frames), 4, 8, 16), np.uint8) for key in MODELS}
    provenance = {
        "scope": "Counterfactual command replay on recorded images, not closed-loop navigation or actual robot motion",
        "source_analysis": str(analysis), "source_provenance": json.loads((analysis / "provenance.json").read_text()),
        "source_input_sha256": {n: source_complete["sha256"][n] for n in ("native-class-masks.npz", "frames.jsonl")},
        "controller": "Existing fcn_replay.FcnDrive -> FcnRoadPlanner -> LocalPathSmoother -> decide_drive",
        "controller_code_sha256": {name: sha(Path(__file__).parent / name) for name in CODE},
        "drive_config": asdict(controllers[0].drive), "local_path_config": asdict(controllers[0].planner.config),
        "requested_simulation_max_speed_mps": speed,
        "speed_scaling": "Existing browser command method multiplies normalized forward speed by requested speed and observed-horizon/turn fraction; no ROS speed settings changed",
        "camera_height_m": height, "camera_pitch_deg": 0, "extrinsics_measured": False,
        "camera_projection": "front90, left180, rear270, right0; vertical FOV90; ASPECT=480/270",
        "adaptation_1024": "Offline compatibility adapter: apply the current fixed front robot region, suppress other ego as unknown, then each 2x2 native block becomes road if >=3/4 road; otherwise unique nonroad mode, ties unknown=0. The fixed planning exclusion is identical to the fixed robot region.",
        "fixed_front_robot_prior": {"revision": FRONT_ROBOT_REVISION, "reference_grid": [8,16],
                                    "columns_zero_based": list(range(*FRONT_ROBOT_COLUMNS)), "rows_zero_based": list(range(*FRONT_ROBOT_ROWS)),
                                    "off_region_ego": "unobserved=0",
                                    "class_id": EGO, "is_learned_prediction": False},
        "fault_injection": False, "map_pose_odom_lidar_input": False,
        "time": "Actual source PTS; planner applies its existing >1s gap reset; missing video bins show commands unavailable",
        "prediction": "A four-second constant-command unicycle arc, reinitialized at the robot origin for each frame; never an accumulated world trajectory",
        "display_action_threshold_radps": .03,
    }
    write_json(out / "provenance.json", provenance)
    for name in CODE:
        shutil.copy2(Path(__file__).parent / name, out / "source" / name)
    shutil.copy2(Path(__file__), out / "source" / Path(__file__).name)
    rows = []
    started = time.monotonic()
    with (out / "controller-trace.jsonl").open("w") as handle, (out / "commands.csv").open("w", newline="") as cf:
        writer = csv.writer(cf)
        writer.writerow(["source_pts", "source_sec", "model", "vx_mps", "vy_mps", "yaw_radps", "action", "reason", "planner_maneuver"])
        for i, frame in enumerate(frames):
            result = {k: frame[k] for k in ("index", "source_pts", "source_time_sec", "source_time_bin")}
            result["controllers"] = {}
            for j, (key, native_key) in enumerate(zip(MODELS, ("fcn512_finetuned", "fcn1024_finetuned"))):
                grid = legacy_grid(masks[native_key][i])
                grids[key][i] = grid
                command, metrics, points = controllers[j].command(grid, frame["source_time_sec"], speed)
                assert math.isfinite(command.vx) and 0 <= command.vx <= speed + 1e-8
                assert command.vy == 0 and abs(command.yaw_rate) <= controllers[j].drive.max_yaw_rps + 1e-8
                record = {"vx_mps": command.vx, "vy_mps": command.vy, "yaw_radps": command.yaw_rate,
                          "reason": command.reason, "action": action(command.vx, command.yaw_rate),
                          "planner": metrics, "local_path_xy_m": points}
                result["controllers"][key] = record
                writer.writerow([frame["source_pts"], frame["source_time_sec"], key, command.vx, command.vy,
                                 command.yaw_rate, record["action"], command.reason, metrics["maneuver"]])
            rows.append(result)
            handle.write(json.dumps(result, allow_nan=False) + "\n")
            if (i + 1) % 500 == 0 or i + 1 == len(frames):
                handle.flush()
                print(f"Controller replay {i + 1}/{len(frames)} ({time.monotonic() - started:.1f}s)", flush=True)
    np.savez_compressed(out / "controller-inputs.npz", pts=masks["pts"], **grids)
    statistics = {}
    for key in MODELS:
        actions = [r["controllers"][key] for r in rows]
        moving = [r for r in actions if r["vx_mps"] > 0]
        statistics[key] = {
            "action_counts": dict(Counter(r["action"] for r in actions)),
            "decision_reasons": dict(Counter(r["reason"] for r in actions)),
            "planner_maneuvers": dict(Counter(r["planner"]["maneuver"] for r in actions)),
            "moving_samples": len(moving), "moving_fraction_of_inferred_samples": len(moving) / len(rows),
            "max_vx_mps": max(r["vx_mps"] for r in actions),
            "mean_vx_when_moving_mps": float(np.mean([r["vx_mps"] for r in moving])) if moving else None,
            "max_abs_yaw_radps": max(abs(r["yaw_radps"]) for r in actions),
        }
    # Representative command cases, selected by a fixed rule, not by driving success.
    picks = []
    for desired in ("forward", "left", "right", "stop"):
        candidates = []
        for i, row in enumerate(rows):
            c = row["controllers"][MODELS[1]]
            if c["action"] != desired:
                continue
            if i + 4 < len(rows) and all(rows[k]["controllers"][MODELS[1]]["action"] == desired for k in range(i, i + 5)):
                candidates.append(i)
        if candidates:
            # First sustained instance with an 8-second separation from other selected cases.
            i = next((i for i in candidates if all(abs(rows[i]["source_time_sec"]-p["source_sec"]) >= 8 for p in picks)), candidates[0])
            picks.append({"action": desired, "source_sec": rows[i]["source_time_sec"], "index": i})
    summary = {"samples": len(rows), "last_source_sec": rows[-1]["source_time_sec"],
               "statistics_scope": "Counts of actual inference observations, excluding missing timestamp bins; not time percentages or driving success rates",
               "controllers": statistics, "representative_cases": picks,
               "trace_wall_sec": time.monotonic() - started}
    write_json(out / "summary.json", summary)
    write_json(out / "trace-complete.json", {"samples": len(rows), "sha256": {
        n: sha(out / n) for n in ("controller-trace.jsonl", "commands.csv", "controller-inputs.npz", "provenance.json", "summary.json")}})
    print(json.dumps(statistics, ensure_ascii=False), flush=True)


class ReplayDisplay:
    """Display local geometry; all forecasts restart at the robot origin."""
    def __init__(self, height, speed):
        self.height = height
        self.labels = {}
        self.font_path = "/System/Library/Fonts/AppleSDGothicNeo.ttc"
        image = Image.new("RGB", (1600, 1080), "#14202b")
        d = ImageDraw.Draw(image)
        font = lambda size: ImageFont.truetype(self.font_path, size)
        d.text((16, 7), "FCN 차도 검출 + 현재 Road 제어기 · 오프라인 명령 재생", font=font(30), fill="white")
        for j, title in enumerate(("512×256 파인튜닝 · 기존 8×16 입력", "1024×512 파인튜닝 → 8×16 호환 입력")):
            x = j * 800
            d.text((x + 12, 55), title, font=font(24), fill="#d9e9ff")
            d.text((x + 12, 502), "로봇 기준 평면도 · 전방 ↑", font=font(20), fill="#d9e9ff")
            d.text((x + 326, 502), f"최대 {speed:.1f} m/s · 회전 ±0.18 rad/s", font=font(20), fill="#d9e9ff")
            d.text((x + 12, 831), "360도 전체 · 어두운 극지방은 미관측", font=font(19), fill="#d9e9ff")
            d.text((x + 623, 955), "제어기 전방 격자", font=font(16), fill="#c1d3e6")
            d.text((x + 623, 980), "회색 = 미관측", font=font(16), fill="#c1d3e6")
            d.text((x + 623, 1005), "초록 = 관측 차도", font=font(16), fill="#c1d3e6")
        d.text((12, 1061), f"파랑=로컬 경로 · 노랑=같은 명령을 4초 유지한 가정 궤적 · 실제 이동 궤적 아님 · 카메라 높이 {height:g} m/수평 추정", font=font(16), fill="#cedded")
        self.base = cv2.cvtColor(np.array(image), cv2.COLOR_RGB2BGR)
        self.fixed_front = cv2.resize(front_robot_mask().astype(np.uint8), (800,400),
                                      interpolation=cv2.INTER_NEAREST).astype(bool)
        ego = np.zeros((4,270,480), np.uint8)
        ego[0] = cv2.resize(front_robot_mask().astype(np.uint8), (480,270), interpolation=cv2.INTER_NEAREST)
        preview = panorama_mask(np.zeros_like(ego),width=400,height=200,forward_heading_deg=90,ego_masks=ego)
        self.fixed_panorama = np.all(preview == (200,70,230),axis=2)
        # A ten-metre, twelve-metre-wide robot-centred view; no site map.
        vv, uu = np.mgrid[:290, :300]
        self.forward = (290 - vv - .5) / 290 * 10
        self.left = (150 - uu - .5) / 300 * 12

    def label(self, canvas, value, xy, size=23, color=(230, 238, 250)):
        key = (value, size, color)
        if key not in self.labels:
            f = ImageFont.truetype(self.font_path, size)
            box = f.getbbox(value)
            im = Image.new("RGB", (box[2] + 6, size + 10), "#14202b")
            ImageDraw.Draw(im).text((2, 0), value, font=f, fill=color)
            self.labels[key] = cv2.cvtColor(np.array(im), cv2.COLOR_RGB2BGR)
        im = self.labels[key]
        x, y = xy
        canvas[y:y + im.shape[0], x:x + im.shape[1]] = im

    @staticmethod
    def text(canvas, value, xy, scale=.58, color=(230, 238, 245)):
        cv2.putText(canvas, value, xy, cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)

    @staticmethod
    def ground_pixels(points):
        points = np.asarray(points)
        return np.rint(np.column_stack((150 - points[:, 1] / 12 * 300,
                                       290 - points[:, 0] / 10 * 290))).astype(np.int32)

    def ground_view(self, observed, path, arc, has_command):
        lab = ground_sample(observed[0], self.forward, self.left, self.height)
        for side, view in ((1, 1), (-1, 3)):
            lateral = side * self.left
            use = lateral > self.forward
            lab[use] = ground_sample(observed[view], lateral[use], -side * self.forward[use], self.height)
        out = np.full((290, 300, 3), (65, 68, 72), np.uint8)
        for c in range(1, 21):
            out[lab == c] = COLORS[c] * .42
        out[lab == ROAD] = (55, 125, 44)
        for dist in (2, 4, 6, 8):
            y = round(290 - dist / 10 * 290)
            cv2.line(out, (0, y), (299, y), (100, 100, 100), 1)
            self.text(out, f"{dist}m", (4, y - 3), .4)
        cv2.line(out, (150, 0), (150, 289), (100, 100, 100), 1)
        if len(path) >= 2:
            cv2.polylines(out, [self.ground_pixels(path)], False, (240, 210, 50), 3, cv2.LINE_AA)
        if has_command:
            uv = self.ground_pixels(arc)
            cv2.polylines(out, [uv], False, (0, 220, 255), 3, cv2.LINE_AA)
            if np.linalg.norm(uv[-1] - uv[-7]) > 1:
                cv2.arrowedLine(out, tuple(uv[-7]), tuple(uv[-1]), (0, 220, 255), 3, cv2.LINE_AA, tipLength=.5)
        cv2.fillPoly(out, [np.array([[150, 272], [143, 286], [157, 286]])], (245, 245, 245))
        return out

    def projected_path(self, frame, points, color, unknown=None):
        p = np.asarray(points, np.float64)
        if p.size == 0:
            return
        p = p[(p[:, 0] > .8) & np.isfinite(p).all(1)]
        if len(p) < 2:
            return
        h, w = frame.shape[:2]
        uv = np.rint(np.column_stack(((.5 - p[:, 1] / (2 * ASPECT * p[:, 0])) * w - .5,
                                      (.5 + self.height / (2 * p[:, 0])) * h - .5))).astype(np.int32)
        for i, (a, b) in enumerate(zip(uv, uv[1:])):
            mid = (a + b) // 2
            if unknown is not None and unknown[mid[1].clip(0, h-1), mid[0].clip(0, w-1)] and i % 2:
                continue
            ok, aa, bb = cv2.clipLine((0, 0, w, h), tuple(a), tuple(b))
            if ok:
                cv2.line(frame, aa, bb, color, 3, cv2.LINE_AA)
        inside = (uv[:, 0] >= 0) & (uv[:, 0] < w) & (uv[:, 1] >= 0) & (uv[:, 1] < h)
        visible = uv[inside]
        if len(visible) >= 3:
            cv2.arrowedLine(frame, tuple(visible[-3]), tuple(visible[-1]), color, 3, cv2.LINE_AA, tipLength=.5)

    def full_panorama_path(self, frame, points, color):
        p = np.asarray(points, np.float64)
        if p.size == 0:
            return
        p = p[p[:, 0] > .8]
        if len(p) < 2:
            return
        lon = np.arctan2(-p[:, 1], p[:, 0]) - np.pi / 2
        lat = np.arctan2(-self.height, np.hypot(p[:, 0], p[:, 1]))
        h, w = frame.shape[:2]
        uv = np.rint(np.column_stack((((lon + np.pi) % (2 * np.pi)) / (2 * np.pi) * w - .5,
                                     (.5 - lat / np.pi) * h - .5))).astype(np.int32)
        for a, b in zip(uv, uv[1:]):
            if abs(a[0] - b[0]) < w / 2:
                cv2.line(frame, tuple(a), tuple(b), color, 2, cv2.LINE_AA)

    def frame(self, original, row, grids, video_time):
        out = self.base.copy()
        source_stamp = row["source_time_sec"] if row is not None else video_time
        self.text(out, f"SOURCE {int(source_stamp)//60:02}:{source_stamp%60:06.3f}", (1160, 34), .72)
        for j, (key, source_col) in enumerate(zip(MODELS, (0, 2))):
            x, source_x = j * 800, source_col * 640
            front = cv2.resize(original[418:738, source_x:source_x + 640], (800, 400))
            panorama = cv2.resize(original[64:384, source_x:source_x + 640], (400, 200))
            if row is None:
                out[92:492, x:x + 800] = front
                out[857:1057, x:x + 400] = panorama
                self.label(out, "원본 프레임 시간 공백 · 제어 명령 없음", (x + 70, 550), 27)
                self.text(out, "NO NEW FRAME / COMMAND UNAVAILABLE", (x + 85, 615), .65)
                continue
            c = row["controllers"][key]
            # The source comparison video keeps its raw historical predictions.
            # Show the current fixed semantic prior separately on this replay.
            front[self.fixed_front] = (.35 * front[self.fixed_front] + .65 * np.array([200,70,230])).astype(np.uint8)
            panorama[self.fixed_panorama] = (.35 * panorama[self.fixed_panorama] + .65 * np.array([200,70,230])).astype(np.uint8)
            obs = observed_classes(grids[key])
            unknown = cv2.resize(((obs[0] == 0) | (obs[0] == BLIND)).astype(np.uint8), (800, 400), interpolation=cv2.INTER_NEAREST).astype(bool)
            yy, xx = np.mgrid[:400, :800]
            hatch = unknown & (((xx + yy) % 17) < 2)
            front[hatch] = (145, 145, 145)
            points = c["local_path_xy_m"]
            arc = command_arc(c["vx_mps"], c["yaw_radps"])
            moving = c["action"] != "stop"
            self.projected_path(front, points, (240, 210, 50), unknown)
            if moving:
                self.projected_path(front, arc, (0, 220, 255), unknown)
            out[92:492, x:x + 800] = front
            state = {"forward":"전진 · 미세 조향", "left":"좌회전 이동", "right":"우회전 이동", "stop":"정지"}[c["action"]]
            color = (255, 145, 135) if not moving else (155, 245, 185)
            self.label(out, state, (x + 12, 101), 30, color)
            out[530:820, x + 8:x + 308] = self.ground_view(obs, points, arc, moving)
            self.text(out, f"v {c['vx_mps']:.2f} m/s   yaw {c['yaw_radps']:+.3f} rad/s", (x + 326, 554), .61)
            self.text(out, "decision: " + c["reason"], (x + 326, 586), .54)
            m = c["planner"]
            self.text(out, "planner: " + m["maneuver"], (x + 326, 615), .54)
            self.text(out, f"side road  L {m.get('left_road_support',0)*100:3.0f}%   R {m.get('right_road_support',0)*100:3.0f}%", (x + 326, 647), .57)
            self.text(out, f"observed path: {m['observed_horizon_m']:.1f} m", (x + 326, 679), .57)
            self.text(out, "centering: " + m["center_status"], (x + 326, 711), .54)
            edges = ["?" if m[z] is None else f"{m[z]:.1f}m" for z in ("left_edge_m", "right_edge_m")]
            self.text(out, "side edges L " + edges[0] + "  R " + edges[1], (x + 326, 743), .54)
            self.text(out, "grey hatch = unobserved arm / cell", (x + 326, 775), .49)
            self.text(out, "yellow = 4s constant-command forecast", (x + 326, 803), .49)
            self.full_panorama_path(panorama, points, (240, 210, 50))
            if moving:
                self.full_panorama_path(panorama, arc, (0, 220, 255))
            out[857:1057, x:x + 400] = panorama
            for k, (name, start_y) in enumerate(zip(("LEFT", "REAR", "RIGHT"), (772, 868, 964))):
                mini = original[start_y:start_y + 96, source_x:source_x + 192]
                y = 865 + k * 64
                out[y:y + 48, x + 415:x + 511] = cv2.resize(mini, (96, 48))
                self.text(out, name, (x + 520, y + 28), .44)
            grid = obs[0]
            rgb = np.full((8, 16, 3), 88, np.uint8)
            seen = (grid > 0) & (grid < 21)
            rgb[seen] = COLORS[grid[seen]]
            out[865:945, x + 624:x + 784] = cv2.resize(rgb, (160, 80), interpolation=cv2.INTER_NEAREST)
        cv2.line(out, (799, 42), (799, 1058), (125, 125, 125), 1)
        return out


def render(analysis, out):
    manifest = json.loads((out / "trace-complete.json").read_text())
    for name, expected in manifest["sha256"].items():
        if sha(out / name) != expected:
            raise ValueError("Modified controller trace: " + name)
    source_complete = json.loads((analysis / "complete.json").read_text())
    video = analysis / "id482-before25m-comparison-5fps.mp4"
    if sha(video) != source_complete["sha256"][video.name]:
        raise ValueError("Changed source comparison video")
    rows = [json.loads(x) for x in (out / "controller-trace.jsonl").read_text().splitlines()]
    summary = json.loads((out / "summary.json").read_text())
    config = json.loads((out / "provenance.json").read_text())
    with np.load(out / "controller-inputs.npz") as archive:
        grids = {key: archive[key] for key in MODELS}
    by_bin = {r["source_time_bin"]: i for i, r in enumerate(rows)}
    display = ReplayDisplay(config["camera_height_m"], config["requested_simulation_max_speed_mps"])
    cv2.setNumThreads(2)
    # Merge brief windows around sustained command examples, plus a late road section.
    windows = [(max(0, math.floor((r["source_sec"] - 4)*5)), min(7500, math.ceil((r["source_sec"] + 10)*5)))
               for r in summary["representative_cases"]]
    windows.append((1376 * 5, 1390 * 5))
    merged = []
    for lo, hi in sorted(windows):
        if merged and lo <= merged[-1][1]:
            merged[-1][1] = max(merged[-1][1], hi)
        else:
            merged.append([lo, hi])
    highlights = {b for lo, hi in merged for b in range(lo, hi)}
    write_json(out / "highlights-windows.json", {"source_windows_sec": [[a/5,b/5] for a,b in merged],
                                                "rule": "First five-observation sustained forward/left/right/stop cases of the 1024 compatibility controller, with four seconds lead and ten seconds follow; overlaps merged; plus 22:56–23:10"})
    selected = {r["source_time_bin"] for r in rows if r["index"] in {p["index"] for p in summary["representative_cases"]}}
    selected.update(min(rows, key=lambda r: abs(r["source_time_sec"]-t))["source_time_bin"] for t in (0,180,780,1380,1499))
    def encoder(path, threads):
        return ["ffmpeg", "-y", "-nostdin", "-v", "error", "-f", "rawvideo", "-pix_fmt", "bgr24", "-s", "1600x1080", "-r", "5", "-i", "pipe:0", "-an", "-c:v", "libx264", "-preset", "veryfast", "-crf", "22", "-threads", str(threads), "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(path)]
    decoder_args = ["ffmpeg", "-nostdin", "-v", "error", "-threads", "2", "-i", str(video), "-map", "0:v:0", "-an", "-fps_mode", "passthrough", "-pix_fmt", "bgr24", "-f", "rawvideo", "pipe:1"]
    start = time.monotonic()
    processed = valid = gaps = 0
    with (out / "render-decode.log").open("w") as dl, (out / "render-encode.log").open("w") as el, (out / "highlights-encode.log").open("w") as hl, (out / "video-timeline.jsonl").open("w") as tl:
        decoder = subprocess.Popen(decoder_args, stdout=subprocess.PIPE, stderr=dl)
        full = subprocess.Popen(encoder(out / "id482-road-controller-overlay-full-25min.mp4", 3), stdin=subprocess.PIPE, stderr=el)
        short = subprocess.Popen(encoder(out / "id482-road-controller-overlay-highlights.mp4", 2), stdin=subprocess.PIPE, stderr=hl)
        try:
            for b in range(7500):
                raw = decoder.stdout.read(1920 * 1080 * 3)
                if len(raw) != 1920 * 1080 * 3:
                    raise ValueError(f"Short comparison video at frame {b}")
                original = np.frombuffer(raw, np.uint8).reshape(1080, 1920, 3)
                i = by_bin.get(b)
                row = rows[i] if i is not None else None
                frame = display.frame(original, row, {k: grids[k][i] for k in MODELS} if i is not None else None, b / 5)
                full.stdin.write(frame.tobytes())
                if b in highlights:
                    short.stdin.write(frame.tobytes())
                if b in selected:
                    assert cv2.imwrite(str(out / "previews" / f"source-{b/5:07.1f}s.jpg"), frame, [cv2.IMWRITE_JPEG_QUALITY, 94])
                tl.write(json.dumps({"video_frame": b, "video_time_sec": b/5, "source_index": i,
                                     "source_pts": row["source_pts"] if row else None,
                                     "command_available": row is not None}) + "\n")
                valid += row is not None
                gaps += row is None
                processed += 1
                if (b+1) % 500 == 0 or b == 7499:
                    print(f"Overlay video {b+1}/7500 frames, {time.monotonic()-start:.1f}s", flush=True)
            if decoder.stdout.read(1):
                raise ValueError("Unexpected trailing video frames")
            decoder.stdout.close()
            assert decoder.wait() == 0
            full.stdin.close()
            short.stdin.close()
            assert full.wait() == short.wait() == 0
        finally:
            for proc in (decoder, full, short):
                if proc.poll() is None:
                    proc.kill()
    assert valid == len(rows) == 6330 and gaps == 1170
    summary["render"] = {"frames": processed, "inferred_source_frames": valid, "unobserved_gap_bins": gaps,
                          "highlight_frames": len(highlights), "highlight_duration_sec": len(highlights)/5,
                          "full_duration_sec": 1500, "resolution": [1600,1080], "fps": 5,
                          "wall_sec": time.monotonic()-start}
    write_json(out / "video-summary.json", summary)
    shutil.copy2(Path(__file__), out / "source" / "render_fcn_control_replay-final.py")
    files = ("id482-road-controller-overlay-full-25min.mp4", "id482-road-controller-overlay-highlights.mp4",
             "video-summary.json", "video-timeline.jsonl", "controller-trace.jsonl", "controller-inputs.npz", "commands.csv")
    write_json(out / "complete.json", {"source_frames": valid, "gap_bins": gaps, "sha256": {n: sha(out/n) for n in files}})
    (out / "index.html").write_text('''<!doctype html><meta charset="utf-8"><title>FCN Road 제어기 영상 재생</title>
<style>body{background:#14202b;color:#eaf2ff;max-width:1500px;margin:24px auto;font:18px/1.7 system-ui}video,img{width:100%}a{color:#8fd4ff}</style>
<h1>FCN Road 제어기 · ID482 25:00 이전</h1>
<p>왼쪽: 512 파인튜닝 / 오른쪽: 1024 파인튜닝을 기존 8×16 제어기 격자로 축소한 호환 실험. 동일한 현재 브라우저 제어기를 최대 2.0 m/s 조건으로 사용했습니다.</p>
<p>파랑: 로컬 경로 · 노랑: 해당 명령을 4초 유지한다고 가정한 궤적 · 회색 사선: 팔 또는 미관측. 실제 촬영 이동이나 폐루프 주행 궤적은 아닙니다. 높이 1.6 m·수평 장착은 추정값입니다.</p>
<h2>약 1분 요약</h2><video controls preload="metadata" src="id482-road-controller-overlay-highlights.mp4"></video>
<h2>전체 25분</h2><video controls preload="metadata" src="id482-road-controller-overlay-full-25min.mp4"></video>
<p><a href="commands.csv">시점별 속도·회전 명령 CSV</a> · <a href="controller-trace.jsonl">제어기 전체 기록</a> · <a href="video-summary.json">요약 통계</a> · <a href="previews/">미리보기 이미지</a></p>''')
    print(f"COMPLETE {out}", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--analysis", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--speed", type=float, default=2.0)
    parser.add_argument("--height", type=float, default=1.6)
    parser.add_argument("--stage", choices=("all", "trace", "render"), default="all")
    args = parser.parse_args()
    if not .1 <= args.speed <= 2 or not 0 < args.height < 5:
        parser.error("speed must be 0.1..2.0m/s and camera height 0..5m")
    if args.stage in ("all", "trace"):
        replay(args.analysis.resolve(), args.output.resolve(), args.speed, args.height)
    if args.stage in ("all", "render"):
        render(args.analysis.resolve(), args.output.resolve())


if __name__ == "__main__":
    main()

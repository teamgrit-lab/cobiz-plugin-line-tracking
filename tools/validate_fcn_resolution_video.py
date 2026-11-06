"""Compare three FCNs on actual 360 source frames, strictly before a time cutoff.

One actual decoded frame per occupied 0.2-second timestamp bin is inferred.
Missing bins are skipped in the review video while original source timestamps
are retained. No inference, road labels, or motion commands are invented.
"""
import argparse
import hashlib
import html
import json
import re
import shutil
import subprocess
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import cv2
import numpy as np
import onnx
import onnxruntime as ort
import torch
from onnx import numpy_helper

from finetune_fcn512 import CityscapesFCN, CLASSES, normalize
from prepare_fcn_resolution_dataset import projection_maps

ORDER = ("FRONT 90", "LEFT 180", "REAR 270", "RIGHT 0")
# BGR. All native classes remain intact; no ego filling or road filtering.
COLORS = np.array([
    [70, 70, 70], [200, 70, 230], [120, 100, 90], [70, 215, 45],
    [30, 180, 250], [190, 120, 120], [156, 102, 102], [153, 153, 190],
    [153, 153, 153], [30, 170, 250], [0, 220, 220], [50, 65, 215],
    [70, 130, 150], [220, 170, 80], [60, 30, 250], [160, 80, 20],
    [100, 60, 20], [100, 100, 20], [120, 80, 40], [180, 50, 150],
    [100, 50, 200],
], np.uint8)


def sha(path):
    with path.open("rb") as fp:
        return hashlib.file_digest(fp, "sha256").hexdigest()


def write_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n")


def load_onnx(path):
    model = CityscapesFCN()
    graph = onnx.load(path)
    state = {x.name: torch.from_numpy(numpy_helper.to_array(x).copy()) for x in graph.graph.initializer}
    model.load_state_dict(state, strict=True)
    return model.eval()


def load_checkpoint(path):
    model = CityscapesFCN()
    state = torch.load(path, map_location="cpu", weights_only=True)
    model.load_state_dict(state["state_dict"], strict=True)
    return model.eval()


def prepare(views, device):
    rgb = np.stack([cv2.cvtColor(v, cv2.COLOR_BGR2RGB) for v in views])
    return normalize(torch.from_numpy(rgb).permute(0, 3, 1, 2).to(device))


@torch.inference_mode()
def predict(model, views, device):
    scores = model(prepare(views, device))
    if not torch.isfinite(scores).all().item():
        raise FloatingPointError("Nonfinite FCN output")
    return (1 + scores[:, 1:].argmax(1)).cpu().numpy().astype(np.uint8)


class Shared1024(torch.nn.Module):
    """Identical frozen layers 1–3 computed once; both original heads retained."""
    def __init__(self, original, finetuned):
        super().__init__()
        for key, value in original.backbone.state_dict().items():
            if not key.startswith("layer4.") and not torch.equal(value, finetuned.backbone.state_dict()[key]):
                raise ValueError(f"Cannot share a changed trunk parameter: {key}")
        self.original, self.finetuned = original, finetuned

    def forward(self, image):
        net = self.original.backbone
        x = net.maxpool(net.relu(net.bn1(net.conv1(image))))
        x = net.layer3(net.layer2(net.layer1(x)))
        return (self.original.classifier(net.layer4(x)),
                self.finetuned.classifier(self.finetuned.backbone.layer4(x)))


@torch.inference_mode()
def predict_pair(model, views, device):
    scores = model(prepare(views, device))
    results = []
    for score in scores:
        if not torch.isfinite(score).all().item():
            raise FloatingPointError("Nonfinite shared FCN output")
        results.append((1 + score[:, 1:].argmax(1)).cpu().numpy().astype(np.uint8))
    return results


def panorama_maps(width=640, height=320):
    yy, xx = np.mgrid[:height, :width]
    lon = ((xx + .5) / width - .5) * 2 * np.pi
    lat = (.5 - (yy + .5) / height) * np.pi
    heading = -lon - np.deg2rad(90)
    sector = np.mod(np.rint(heading / (np.pi / 2)).astype(int), 4)
    delta = (heading - sector * np.pi / 2 + np.pi) % (2 * np.pi) - np.pi
    u = .5 - np.tan(delta) / (2 * (480 / 270))
    v = .5 - np.tan(lat) / (2 * np.cos(delta))
    visible = (u >= 0) & (u < 1) & (v >= 0) & (v < 1)
    return u, v, sector, visible


PMAPS = panorama_maps()


def erp_labels(masks):
    u, v, sector, visible = PMAPS
    out = np.full(u.shape, 255, np.uint8)
    h, w = masks.shape[1:]
    for j in range(4):
        use = visible & (sector == j)
        out[use] = masks[j, np.minimum((v[use] * h).astype(int), h - 1),
                              np.minimum((u[use] * w).astype(int), w - 1)]
    return out


def overlay(raw, mask):
    out = raw.copy()
    seen = mask != 255
    out[seen] = (out[seen] * .58 + COLORS[mask[seen]] * .42).astype(np.uint8)
    out[~seen] = (out[~seen] * .58).astype(np.uint8)
    return out


def text(image, value, xy, scale=.55, color=(240, 240, 240)):
    cv2.putText(image, value, xy, cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)


def composite(panorama, views, masks, titles, stamp):
    canvas = np.full((1080, 1920, 3), (22, 27, 32), np.uint8)
    raw_erp = cv2.resize(panorama, (640, 320), interpolation=cv2.INTER_AREA)
    for j, (pred, title) in enumerate(zip(masks, titles)):
        x = j * 640
        text(canvas, title, (x + 12, 26), .64)
        text(canvas, f"ID482 {int(stamp)//60:02}:{stamp%60:06.3f} | inference at actual PTS", (x + 12, 49), .48)
        canvas[64:384, x:x + 640] = overlay(raw_erp, erp_labels(pred))
        text(canvas, "360 PANORAMA | dark caps = unobserved", (x + 10, 405), .52)
        raw = cv2.resize(views[0], (640, 320), interpolation=cv2.INTER_AREA)
        labels = cv2.resize(pred[0], (640, 320), interpolation=cv2.INTER_NEAREST)
        canvas[418:738, x:x + 640] = overlay(raw, labels)
        text(canvas, "FRONT 90 deg | robot pixels are NOT filled", (x + 10, 759), .52)
        for k in (1, 2, 3):
            y = 772 + (k - 1) * 96
            raw = cv2.resize(views[k], (192, 96), interpolation=cv2.INTER_AREA)
            labels = cv2.resize(pred[k], (192, 96), interpolation=cv2.INTER_NEAREST)
            canvas[y:y + 96, x:x + 192] = overlay(raw, labels)
            text(canvas, ORDER[k], (x + 207, y + 30), .55)
            text(canvas, f"road cells {int((pred[k]==3).sum())}/{pred[k].size}", (x + 207, y + 55), .5)
        cv2.line(canvas, (x + 639, 0), (x + 639, 1079), (100, 105, 110), 1)
    return canvas


def read_frame(pipe, size):
    raw = pipe.read(size)
    if not raw:
        return None
    if len(raw) != size:
        raise ValueError(f"Short decoded frame: {len(raw)} != {size}")
    return np.frombuffer(raw, np.uint8).reshape(1440, 2880, 3)


@torch.inference_mode()
def check_parity(models, paths, views, device):
    results = []
    for model, path, width in zip(models, paths, (512, 1024, 1024)):
        frame = views[width][0]
        inp = prepare([frame], device)
        actual = model(inp).cpu().numpy()
        options = ort.SessionOptions()
        options.intra_op_num_threads = 3
        options.log_severity_level = 3
        session = ort.InferenceSession(str(path), options, providers=["CPUExecutionProvider"])
        expected = session.run(["output_0"], {"input_0": inp.cpu().numpy()})[0]
        np.testing.assert_allclose(actual, expected, rtol=5e-4, atol=5e-4)
        different = int((actual[:, 1:].argmax(1) != expected[:, 1:].argmax(1)).sum())
        if different:
            raise ValueError(f"MPS / ONNX class mismatch: {path}: {different}")
        results.append({"onnx": str(path), "max_logit_error": float(np.abs(actual - expected).max()),
                        "different_cells": different, "input_shape": list(inp.shape)})
    return results


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--video", type=Path, required=True)
    parser.add_argument("--run1024", type=Path, required=True)
    parser.add_argument("--base1024", type=Path, required=True)
    parser.add_argument("--run512", type=Path, required=True)
    parser.add_argument("--timestamps", type=Path, required=True,
                        help="Verified actual source PTS manifest from the earlier 5Hz dataset")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--cutoff", type=float, default=1500)
    parser.add_argument("--device", default="mps")
    args = parser.parse_args()
    out = args.output.resolve()
    if out.exists():
        raise FileExistsError(out)
    out.mkdir(parents=True)
    (out / "images-5fps").mkdir()
    (out / "previews").mkdir()
    torch.set_num_threads(3)
    cv2.setNumThreads(2)
    rows = [json.loads(x) for x in args.timestamps.read_text().splitlines()]
    rows = [r for r in rows if r["recording"] == "ID482" and r["source_time_sec"] < args.cutoff]
    source_sha = sha(args.video)
    if {r["source_sha256"] for r in rows} != {source_sha}:
        raise ValueError("Video identity differs from the verified source PTS manifest")
    assert len({int(r["source_time_sec"] * 5 + 1e-8) for r in rows}) == len(rows)
    assert all(a["source_pts"] < b["source_pts"] for a, b in zip(rows, rows[1:]))
    device = torch.device(args.device)
    models = [load_checkpoint(args.run512 / "best.pt"), load_onnx(args.base1024),
              load_checkpoint(args.run1024 / "best.pt")]
    pair = Shared1024(models[1], models[2]).eval().to(device)
    models = [m.to(device) for m in models]
    paths = [args.run512 / "fcn_resnet18.onnx", args.base1024, args.run1024 / "fcn_resnet18.onnx"]
    titles = ["512x256 FINETUNED", "1024x512 ORIGINAL", "1024x512 FINETUNED"]
    maps = {w: projection_maps(w, w // 2) for w in (512, 1024)}
    sample = "isnan(prev_selected_t)+gt(floor(5*t+0.00000001),floor(5*prev_selected_t+0.00000001))"
    decode_command = ["ffmpeg", "-nostdin", "-hide_banner", "-nostats", "-v", "info", "-hwaccel", "videotoolbox",
                      "-i", str(args.video), "-t", str(args.cutoff), "-map", "0:v:0", "-an", "-vf",
                      f"select='lt(t,{args.cutoff})*({sample})',showinfo=checksum=0", "-fps_mode", "passthrough",
                      "-pix_fmt", "bgr24", "-threads", "1", "-f", "rawvideo", "pipe:1"]
    encode_command = ["ffmpeg", "-y", "-nostdin", "-v", "error", "-f", "rawvideo", "-pix_fmt", "bgr24",
                      "-s", "1920x1080", "-r", "5", "-i", "pipe:0", "-an", "-c:v", "libx264",
                      "-preset", "veryfast", "-crf", "22", "-threads", "3", "-pix_fmt", "yuv420p",
                      "-movflags", "+faststart", str(out / "id482-before25m-comparison-5fps.mp4")]
    write_json(out / "provenance.json", {
        "source": str(args.video), "source_sha256": source_sha, "cutoff_exclusive_sec": args.cutoff,
        "sampling": "first actual decoded frame in each occupied 0.2s PTS bin; no repeated inference",
        "models": [{"title": t, "path": str(p), "sha256": sha(p)} for t, p in zip(titles, paths)],
        "annotations": "Inherited partial visual labels; ID482 is the FCN training recording, not a blind test",
        "video": "Actual frames only; missing bins skipped; original source times displayed; no audio",
        "missing_source_policy": "skip",
        "projection": "direct ERP perspective remap, front=90deg, left=180, rear=270, right=0; vertical FOV90",
        "labels": "native class logits argmax over 1..20, nearest cell upsampling; no ego fill, smoothing or planner",
        "optimization": "Identical frozen layers 1-3 shared between the two 1024 models; separate original layer4 and classifiers; same FP32 precision. Image encoding overlaps inference.",
        "decode_command": decode_command, "encode_command": encode_command,
        "class_color_bgr": {c: COLORS[i].tolist() for i, c in enumerate(CLASSES)},
    })
    predictions = [np.empty((len(rows), 4, w // 64, w // 32), np.uint8) for w in (512, 1024, 1024)]
    measurements, preview_records = [], []
    # These fixed, uniformly spaced preview targets are chosen before seeing outputs.
    preview_indices = {int(np.argmin(np.abs(np.array([r["source_time_sec"] for r in rows]) - t)))
                       for t in (0, 60, 180, 300, 420, 540, 660, 780, 900, 1020, 1140, 1260, 1380, 1499)}
    started = time.monotonic()
    next_bin = gap_bins = 0
    trace = (out / "frames.jsonl").open("w")
    def render_frame(i, row, panorama, render_views, outputs, timings):
        nonlocal next_bin, gap_bins
        stamp = row["source_time_sec"]
        canvas = composite(panorama, render_views, outputs, titles, stamp)
        image_name = f"{i:06d}-pts-{row['source_pts']:010d}.jpg"
        if not cv2.imwrite(str(out / "images-5fps" / image_name), canvas, [cv2.IMWRITE_JPEG_QUALITY, 88]):
            raise OSError(image_name)
        occupied_bin = int(stamp * 5 + 1e-8)
        if occupied_bin < next_bin:
            raise ValueError("Source timestamp bins must be strictly increasing")
        gap_bins += occupied_bin - next_bin
        encoder.stdin.write(canvas.tobytes())
        next_bin = occupied_bin + 1
        result = {"index": i, "source_pts": row["source_pts"], "source_time_sec": stamp,
                  "source_present": True, "output_frame_index": i, "output_time_sec": i / 5.,
                  "source_time_bin": occupied_bin, "image": "images-5fps/" + image_name,
                  "recording_seen_in_training": True,
                  "inference_ms_groups": timings,
                  "road_fraction_by_model_and_view": [[float((p == 3).mean()) for p in pred] for pred in outputs]}
        measurements.append(result)
        trace.write(json.dumps(result) + "\n")
        if i in preview_indices:
            target = out / "previews" / image_name
            shutil.copy2(out / "images-5fps" / image_name, target)
            preview_records.append({**result, "preview": str(target.relative_to(out))})
        if (i + 1) % 100 == 0 or i == len(rows) - 1:
            trace.flush()
            print(json.dumps({"processed": i + 1, "total": len(rows), "source_sec": stamp,
                              "wall_sec": round(time.monotonic() - started, 1)}, ensure_ascii=False), flush=True)

    with (out / "decode.log").open("w") as dec_log, (out / "encode.log").open("w") as enc_log:
        decoder = subprocess.Popen(decode_command, stdout=subprocess.PIPE, stderr=dec_log)
        encoder = subprocess.Popen(encode_command, stdin=subprocess.PIPE, stderr=enc_log)
        render_pool = ThreadPoolExecutor(max_workers=1)
        pending = None
        try:
            for i, row in enumerate(rows):
                panorama = read_frame(decoder.stdout, 2880 * 1440 * 3)
                if panorama is None:
                    raise ValueError(f"Decoder stopped early: {i}/{len(rows)}")
                views = {w: [cv2.remap(panorama, *mp, cv2.INTER_LINEAR, borderMode=cv2.BORDER_WRAP)
                             for mp in maps[w]] for w in maps}
                if i == 0:
                    write_json(out / "onnx-mps-parity.json", check_parity(models, paths, views, device))
                tic = time.monotonic()
                first = predict(models[0], views[512], device)
                first_ms = (time.monotonic() - tic) * 1000
                tic = time.monotonic()
                combined = predict_pair(pair, views[1024], device)
                pair_ms = (time.monotonic() - tic) * 1000
                outputs = [first, *combined]
                if i == 0:
                    for independent, optimized in zip(models[1:], combined):
                        np.testing.assert_array_equal(predict(independent, views[1024], device), optimized)
                    write_json(out / "shared-backbone-parity.json", {
                        "unchanged_trunk_parameters_verified": True,
                        "both_outputs_pixel_equal_to_independent_models": True,
                        "precision": "float32",
                    })
                timings = {"512_finetuned": first_ms, "1024_pair_shared": pair_ms}
                for j, pred in enumerate(outputs):
                    predictions[j][i] = pred
                if pending is not None:
                    pending.result()
                pending = render_pool.submit(render_frame, i, row, panorama, views[1024], outputs, timings)
            if pending is not None:
                pending.result()
            if read_frame(decoder.stdout, 2880 * 1440 * 3) is not None:
                raise ValueError("Decoder emitted extra frames not present in verified PTS manifest")
            decoder.stdout.close()
            if decoder.wait() != 0:
                raise RuntimeError("FFmpeg decode failed")
            gap_bins += max(0, int(args.cutoff * 5) - next_bin)
            encoder.stdin.close()
            if encoder.wait() != 0:
                raise RuntimeError("FFmpeg encode failed")
        finally:
            render_pool.shutdown(wait=True)
            trace.close()
            if decoder.poll() is None:
                decoder.kill()
            if encoder.poll() is None:
                encoder.kill()
    log_text = (out / "decode.log").read_text(errors="replace")
    decoded_pts = [int(x) for x in re.findall(r"\bn:\s*\d+\s+pts:\s*(-?\d+)\s+pts_time:", log_text)]
    expected_pts = [r["source_pts"] for r in rows]
    if decoded_pts != expected_pts:
        raise ValueError("Source PTS differs from reference. Results are not complete.")
    np.savez_compressed(out / "native-class-masks.npz", pts=np.array(expected_pts, np.int64),
                        times=np.array([r["source_time_sec"] for r in rows]),
                        fcn512_finetuned=predictions[0], fcn1024_original=predictions[1],
                        fcn1024_finetuned=predictions[2])
    latency = np.array([[r["inference_ms_groups"][name] for name in ("512_finetuned", "1024_pair_shared")]
                        for r in measurements])
    summary = {"samples": len(rows), "views_per_model": len(rows) * 4,
               "first_source_sec": rows[0]["source_time_sec"], "last_source_sec": rows[-1]["source_time_sec"],
               "cutoff_exclusive_sec": args.cutoff, "source_pts_exact_match": True,
               "source_timestamp_gap_bins": gap_bins, "source_bins": int(args.cutoff * 5),
               "missing_source_policy": "skip", "video_frames": len(rows),
               "video_duration_sec": len(rows) / 5.,
               "wall_seconds": time.monotonic() - started,
               "inference_ms_groups": {title: {"median": float(np.median(latency[:, j])),
                                               "p95": float(np.percentile(latency[:, j], 95))}
                                       for j, title in enumerate(("512_finetuned", "1024_pair_shared"))},
               "scope": "Recorded-video perception review. ID482 includes training scenes. No physical or closed-loop driving validation.",
               "previews": preview_records}
    write_json(out / "summary.json", summary)
    write_json(out / "complete.json", {"source_pts_exact_match": True, "samples": len(rows),
                                       "sha256": {n: sha(out / n) for n in ("summary.json", "provenance.json",
                                             "frames.jsonl", "native-class-masks.npz", "id482-before25m-comparison-5fps.mp4")}})
    images = "".join(f'<p>{r["source_time_sec"]:.3f}s</p><img loading="lazy" src="{html.escape(r["preview"])}">'
                     for r in preview_records)
    (out / "index.html").write_text('<!doctype html><meta charset="utf-8"><title>FCN 1024 차도 검출 비교</title>'
        '<style>body{background:#111820;color:#e9eef5;font:17px system-ui;max-width:1500px;margin:24px auto}'
        'img,video{width:100%;margin:8px 0}a{color:#8fdaff}</style>'
        '<h1>FCN 1024×512 파인튜닝 · ID482 25:00 이전</h1>'
        '<p>왼쪽: 512 파인튜닝 / 가운데: 1024 기본 / 오른쪽: 1024 파인튜닝</p>'
        '<p>초록: 차도 · 분홍: 로봇 · 빨강: 식생 · 노랑: 인도 · 보라: 건물. 어두운 극지방: 미관측.</p>'
        '<p>원본 PTS 기준 5 FPS. 원본 프레임이 없는 구간은 건너뛰고 실제 프레임만 연결합니다. 화면 시각은 원본 기준이며 영상 재생 위치는 output_frame_index/output_time_sec로 기록합니다. ID482는 학습 장면을 포함합니다.</p>'
        '<video controls preload="metadata" src="id482-before25m-comparison-5fps.mp4"></video>'
        '<p><a href="images-5fps/">전체 5 FPS 이미지</a> · <a href="summary.json">추론 기록</a></p>' + images)
    print(f"COMPLETE {out}", flush=True)


if __name__ == "__main__":
    main()

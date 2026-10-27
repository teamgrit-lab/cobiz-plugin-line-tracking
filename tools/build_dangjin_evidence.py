#!/usr/bin/env python3
"""Extract small, traceable camera/LiDAR samples without ROS or a full bag scan.

Inputs are read-only. Point clouds retain their recorded sensor frame; this is
not SLAM, deskewing, colorization, or registration to the supplied site plan.
"""
from __future__ import annotations

import argparse
from collections import Counter
import json
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory

from lidar_common import cloud_xyz
from rosbag_mcap_to_mp4 import decode_video_frame

CAMERA = "/a2/front_camera/res_360p/image_raw"
LIDAR = "/livox/lidar"


def first_message(reader, topic, timestamp):
    # Sensor gaps can exceed two seconds even when other topics are healthy.
    # Seek to the first actual sample after the target within this split file.
    return next(reader.iter_decoded_messages(topics=[topic], start_time=timestamp), None)


def extract(source: Path, output: Path) -> dict:
    files = sorted(p for p in source.glob("*.mcap") if not p.name.startswith("."))
    if not files:
        raise ValueError(f"No MCAP files in {source}")
    output.mkdir(parents=True, exist_ok=True)
    inventory, frames, clouds, counts = [], [], [], Counter()
    start, end = None, None
    contact_images = []
    for index, path in enumerate(files):
        with path.open("rb") as stream:
            reader = make_reader(stream, decoder_factories=[DecoderFactory()])
            summary = reader.get_summary()
            if summary is None or summary.statistics is None:
                raise ValueError(f"Indexed MCAP summary required: {path}")
            st = summary.statistics
            start = st.message_start_time if start is None else min(start, st.message_start_time)
            end = st.message_end_time if end is None else max(end, st.message_end_time)
            topics = {summary.channels[k].topic: v for k, v in st.channel_message_counts.items()}
            counts.update(topics)
            inventory.append({"file": path.name, "bytes": path.stat().st_size,
                              "start_ns": str(st.message_start_time),
                              "end_ns": str(st.message_end_time), "topics": topics})
            for sample, fraction in enumerate((0.1, 0.4, 0.7, 0.9)):
                target = st.message_start_time + int((st.message_end_time - st.message_start_time) * fraction)
                entry = first_message(reader, CAMERA, target)
                if entry is None:
                    continue
                frame = decode_video_frame(entry.decoded_message, entry.schema.name, entry.message.log_time)
                modes = {"rgb24": "RGB", "bgr24": "BGR", "gray": "L", "rgba": "RGBA", "bgra": "BGRA"}
                image = Image.frombytes("RGB" if frame.input_format not in ("gray", "rgba", "bgra") else
                                        ("L" if frame.input_format == "gray" else "RGBA"),
                                        (frame.width, frame.height), frame.payload,
                                        "raw", modes[frame.input_format]).convert("RGB")
                name = f"camera-{index:02d}-{sample:02d}.jpg"
                image.save(output / name, quality=87)
                frames.append({"file": path.name, "image": name,
                               "timestamp_ns": str(entry.message.log_time),
                               "frame_id": entry.decoded_message.header.frame_id,
                               "width": frame.width, "height": frame.height})
                thumb = image.resize((320, 180))
                tile = Image.new("RGB", (320, 202), "#13232c")
                tile.paste(thumb, (0, 0))
                ImageDraw.Draw(tile).text((8, 184), f"part {index} / sample {sample} / {(entry.message.log_time-start)/1e9:.1f}s", fill="white")
                contact_images.append(tile)
            # One independent raw scan near the midpoint of every split file.
            target = (st.message_start_time + st.message_end_time) // 2
            entry = first_message(reader, LIDAR, target)
            if entry is not None:
                xyz = cloud_xyz(entry.decoded_message)
                original_count = len(xyz)
                # Deterministic spread in acquisition order, no fabricated points.
                xyz = xyz[np.linspace(0, len(xyz) - 1, min(6000, len(xyz)), dtype=int)]
                name = f"lidar-{index:02d}.json"
                (output / name).write_text(json.dumps(np.round(xyz, 3).tolist(), separators=(",", ":")))
                clouds.append({"file": path.name, "points": name,
                               "timestamp_ns": str(entry.message.log_time),
                               "frame_id": entry.decoded_message.header.frame_id,
                               "original_points": original_count, "displayed_points": len(xyz)})
            print(f"Extracted {path.name}", flush=True)
    for item in frames + clouds:
        item["elapsed_s"] = round((int(item["timestamp_ns"]) - start) / 1e9, 3)
    manifest = {"schema_version": 1, "recorded_date": "2026-10-21",
                "duration_s": (end-start)/1e9, "message_count": sum(counts.values()),
                "topics": dict(counts), "files": inventory, "frames": frames, "clouds": clouds,
                "registration": "none", "sampling": "4 camera frames and 1 independent LiDAR scan per file",
                "limitations": ["No recorded TF, odometry, GPS or global map topics.",
                                "Camera frames and LiDAR scans are independently sampled, not synchronized.",
                                "LiDAR remains in the sensor frame, without motion compensation or site alignment."]}
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + "\n")
    sheet = Image.new("RGB", (1280, 202 * ((len(contact_images)+3)//4)), "#13232c")
    for i, tile in enumerate(contact_images):
        sheet.paste(tile, ((i % 4)*320, (i//4)*202))
    sheet.save(output / "contact-sheet.jpg", quality=90)
    return manifest


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    extract(args.input.expanduser().resolve(), args.output.expanduser().resolve())

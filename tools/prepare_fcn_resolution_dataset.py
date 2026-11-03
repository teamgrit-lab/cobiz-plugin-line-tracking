"""Reproject existing partial FCN annotations at a new model input resolution.

RGB comes directly from the saved full-resolution ERP panorama. Existing targets
use nearest-neighbour scaling, preserving all unknown pixels and split membership;
this does not create newly annotated high-resolution boundaries.
"""
import argparse
import hashlib
import json
import shutil
from pathlib import Path

import cv2
import numpy as np


def sha(path):
    with path.open("rb") as fp:
        return hashlib.file_digest(fp, "sha256").hexdigest()


def projection_maps(width, height):
    v, u = np.mgrid[:height, :width]
    right = ((u + .5) / width - .5) * 480 / 135
    up = (.5 - (v + .5) / height) * 2
    latitude = np.arctan2(up, np.sqrt(1 + right ** 2))
    y = ((.5 - latitude / np.pi) * 1440 - .5).astype(np.float32)
    return tuple(((((np.arctan2(right, 1) - np.deg2rad(yaw) + np.pi) % (2 * np.pi))
                    * 2880 / (2 * np.pi) - .5).astype(np.float32), y)
                 for yaw in (90, 180, 270, 360))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--width", type=int, choices=[512, 1024], default=1024)
    args = parser.parse_args()
    root, out = args.source.resolve(), args.output.resolve()
    if out.exists():
        raise FileExistsError(f"Refusing to overwrite dataset: {out}")
    schema = json.loads((root / "schema.json").read_text())
    old_h, old_w = schema["shape"]
    width, height = args.width, args.width // 2
    maps = projection_maps(width, height)
    old_maps = projection_maps(old_w, old_h)
    names = ("front", "left", "rear", "right")
    original = {}
    split_lists = {}
    for split in ("train", "validation"):
        for suffix in ("", "-binary"):
            name = f"{split}{suffix}.jsonl"
            rows = [json.loads(x) for x in (root / "splits" / name).read_text().splitlines()]
            split_lists[name] = rows
            for row in rows:
                if row["id"] in original:
                    assert original[row["id"]] == row
                original[row["id"]] = row
    assert not ({r["recording_group"] for r in split_lists["train.jsonl"]}
                & {r["recording_group"] for r in split_lists["validation.jsonl"]})
    out.mkdir(parents=True)
    transformed = {}
    hashes = {}
    for index, row in enumerate(original.values()):
        assert row["annotation_status"] == "visual_partial"
        assert row["clip"] != "ID482" or row["pts_time_sec"] < 1500
        panorama = cv2.imread(str(root / row["panorama"]))
        assert panorama is not None and panorama.shape == (1440, 2880, 3)
        j = names.index(row["view"])
        old_rgb = cv2.remap(panorama, *old_maps[j], cv2.INTER_LINEAR, borderMode=cv2.BORDER_WRAP)
        assert np.array_equal(old_rgb, cv2.imread(str(root / row["image"]))), row["id"]
        rgb = cv2.remap(panorama, *maps[j], cv2.INTER_LINEAR, borderMode=cv2.BORDER_WRAP)
        for key in ("image", "semantic_label", "binary_road_label", "panorama"):
            if not row.get(key):
                continue
            src, dest = root / row[key], out / row[key]
            dest.parent.mkdir(parents=True, exist_ok=True)
            if key == "panorama":
                if not dest.exists():
                    shutil.copy2(src, dest)
            else:
                values = rgb if key == "image" else cv2.resize(
                    cv2.imread(str(src), cv2.IMREAD_UNCHANGED), (width, height), interpolation=cv2.INTER_NEAREST)
                assert cv2.imwrite(str(dest), values, [cv2.IMWRITE_PNG_COMPRESSION, 3])
                assert np.array_equal(values, cv2.imread(str(dest), cv2.IMREAD_UNCHANGED))
            hashes[row[key]] = {"parent_sha256": sha(src), "sha256": sha(dest)}
        transformed[row["id"]] = {k: v for k, v in row.items() if k not in ("proposal", "ego_body_prior")}
        if index % 50 == 0:
            print(f"Reprojected {index + 1}/{len(original)} reviewed views", flush=True)
    (out / "splits").mkdir()
    for name, rows in split_lists.items():
        (out / "splits" / name).write_text("".join(json.dumps(transformed[r["id"]]) + "\n" for r in rows))
    schema["shape"] = [height, width]
    schema["resolution_derivation"] = {
        "parent_dataset": str(root), "parent_shape": [old_h, old_w],
        "rgb": "direct bilinear perspective projection from saved 2880x1440 panorama",
        "targets": "nearest-neighbour scaling of existing partial labels; no finer annotation added",
        "annotation_status": "visual_partial; Codex visual polygons, not independently verified human ground truth",
    }
    (out / "schema.json").write_text(json.dumps(schema, ensure_ascii=False, indent=2) + "\n")
    (out / "manifest.jsonl").write_text("".join(json.dumps(r) + "\n" for r in transformed.values()))
    (out / "provenance").mkdir()
    for name in ("sources.json", "stats.json"):
        shutil.copy2(root / name, out / "provenance" / name)
    shutil.copy2(Path(__file__), out / "provenance" / Path(__file__).name)
    report = {"views": len(original), "shape": [height, width], "split_disjoint": True,
              "all_parent_projections_pixel_equal": True, "all_outputs_reloaded_pixel_equal": True,
              "targets_preserve_parent_unknown_regions": True, "parent_dataset": str(root),
              "split_counts": {n: len(r) for n, r in split_lists.items()}, "files": hashes}
    (out / "verification.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    print(f"Dataset complete: {out}", flush=True)


if __name__ == "__main__":
    main()

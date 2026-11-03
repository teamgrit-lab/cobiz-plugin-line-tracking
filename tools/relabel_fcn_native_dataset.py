"""Build native-resolution road supervision from visual polygon annotations.

Road polygons use source-image pixel coordinates, never model predictions. The
nonroad semantic context is inherited explicitly; road labels are replaced.
Uncertain boundaries and robot/object occlusions remain ignored. Existing
recording-group splits and source images are preserved without overwriting them.
"""
import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shutil

import cv2
import numpy as np


def digest(path):
    with path.open("rb") as source:
        return hashlib.file_digest(source, "sha256").hexdigest()


def write_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n")


def polygon_mask(polygons, shape):
    mask = np.zeros(shape, np.uint8)
    for points in polygons:
        polygon = np.asarray(points)
        if (polygon.ndim != 2 or polygon.shape[0] < 3 or polygon.shape[1] != 2
                or not np.isfinite(polygon).all() or np.any(polygon != np.floor(polygon))
                or np.any(polygon < 0) or np.any(polygon[:, 0] >= shape[1])
                or np.any(polygon[:, 1] >= shape[0])):
            raise ValueError(f"Invalid native polygon: {points}")
        cv2.fillPoly(mask, [polygon.astype(np.int32)], 1)
    return mask.astype(bool)


def safe_path(root, value):
    path = Path(value)
    if path.is_absolute() or ".." in path.parts:
        raise ValueError(f"Invalid dataset path {value}")
    return root / path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", required=True, type=Path)
    parser.add_argument("--annotations", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    root, out = args.source.resolve(), args.output.resolve()
    if out.exists():
        raise FileExistsError(f"Refusing to overwrite dataset: {out}")
    schema = json.loads((root / "schema.json").read_text())
    shape = tuple(schema["shape"])
    if shape != (512, 1024):
        raise ValueError("This annotation revision requires native 1024x512 images")
    annotations = json.loads(args.annotations.read_text())
    if annotations["shape"] != list(shape) or annotations["model_predictions_used"]:
        raise ValueError("Expected independently specified native visual polygons")
    rows = [json.loads(line) for line in (root / "manifest.jsonl").read_text().splitlines()]
    ids = [row["id"] for row in rows]
    if len(set(ids)) != len(ids) or set(ids) != set(annotations["views"]):
        raise ValueError("Annotations must cover exactly every unique source view")
    groups = {split: {r["recording_group"] for r in rows if r["split"] == split}
              for split in ("train", "validation")}
    if groups["train"] & groups["validation"]:
        raise ValueError("Recording groups overlap between train and validation")
    for row in rows:
        if row["recording_group"] == "ID482" and row["pts_time_sec"] >= 1500:
            raise ValueError("Excluded ID482 footage at or after 25:00")
    out.mkdir(parents=True)
    for name in ("annotations", "splits", "provenance", "previews"):
        (out / name).mkdir()
    shutil.copy2(args.annotations, out / "annotations/native-polygons.json")
    shutil.copy2(Path(__file__), out / "provenance" / Path(__file__).name)
    report = {"views": [], "split_disjoint": True, "source": str(root),
              "annotation_sha256": digest(args.annotations), "shape": list(shape),
              "annotation_status": annotations["annotation_status"],
              "model_predictions_used": False, "class_context": "parent partial semantic context only"}
    revised = []
    for row in rows:
        note = annotations["views"][row["id"]]
        if digest(safe_path(root, row["image"])) != note["source_image_sha256"]:
            raise ValueError(f"Annotation image hash mismatch: {row['id']}")
        image = cv2.imread(str(safe_path(root, row["image"])))
        old = cv2.imread(str(safe_path(root, row["semantic_label"])), cv2.IMREAD_GRAYSCALE)
        if image is None or image.shape != (*shape, 3) or old is None or old.shape != shape:
            raise ValueError(f"Invalid source image or mask: {row['id']}")
        road = polygon_mask(note["road_polygons_xy"], shape)
        ignored = polygon_mask(note["ignore_polygons_xy"], shape)
        # Preserve occlusions conservatively; do not guess road behind the robot.
        if note.get("inherited_robot_ignore_xy"):
            inherited = polygon_mask([note["inherited_robot_ignore_xy"]], shape).astype(np.uint8)
            ignored |= cv2.dilate(inherited, np.ones((9, 9), np.uint8)).astype(bool)
        if road.any():
            band = cv2.morphologyEx(road.astype(np.uint8), cv2.MORPH_GRADIENT,
                                    np.ones((7, 7), np.uint8)).astype(bool)
            ignored |= band
        ignored[-8:, :] = True  # ERP stitch/robot edge uncertainty.
        if row["view"] == "front":
            ignored[-24:, :] = True
        semantic = old.copy()
        semantic[(semantic == 3) | (semantic == 1)] = 255
        binary = np.zeros(shape, np.uint8)
        semantic[road] = 3
        binary[road] = 1
        # Existing known foreground objects are never turned into road labels.
        ignored |= np.isin(old, [14, 15, 16, 17, 18, 19, 20])
        semantic[ignored] = 255
        binary[ignored] = 255
        if row["view"] == "front":
            # Same angular region as the current controller: reference 8x16,
            # central two columns and bottom three rows. Outside it, visible
            # robot geometry is ignored, not assigned the robot class.
            semantic[320:512, 448:576] = 1
            binary[320:512, 448:576] = 255
        changed = dict(row, annotation_status="visual_partial",
                       annotation_revision=annotations["revision"],
                       annotation_method="native_1024_visual_polygons",
                       binary_road_label=f"labels_road_binary/{row['clip']}/{row['id']}.png")
        for key in ("image", "panorama"):
            src, dest = safe_path(root, row[key]), safe_path(out, changed[key])
            dest.parent.mkdir(parents=True, exist_ok=True)
            if not dest.exists():
                shutil.copy2(src, dest)
        for key, values in (("semantic_label", semantic), ("binary_road_label", binary)):
            path = safe_path(out, changed[key])
            path.parent.mkdir(parents=True, exist_ok=True)
            if not cv2.imwrite(str(path), values, [cv2.IMWRITE_PNG_COMPRESSION, 3]):
                raise IOError(path)
            if not np.array_equal(cv2.imread(str(path), 0), values):
                raise ValueError(f"PNG round-trip mismatch: {path}")
        valid = binary != 255
        if not valid.any() or not np.array_equal(semantic[valid] == 3, binary[valid] == 1):
            raise ValueError(f"Inconsistent road supervision: {row['id']}")
        assert np.array_equal(image, cv2.imread(str(out / changed["image"])))
        report["views"].append({"id": row["id"], "split": row["split"],
                                "road_pixels": int((binary == 1).sum()),
                                "nonroad_pixels": int((binary == 0).sum()),
                                "ignored_pixels": int((binary == 255).sum()),
                                "parent_semantic_pixels_changed": int((semantic != old).sum()),
                                "old_grass_pixels_now_road": int(((old == 11) & (binary == 1)).sum()),
                                "old_building_pixels_now_road": int(((old == 5) & (binary == 1)).sum()),
                                "image_sha256": digest(out / changed["image"]),
                                "semantic_sha256": digest(out / changed["semantic_label"]),
                                "binary_sha256": digest(out / changed["binary_road_label"])})
        revised.append(changed)
    for split in ("train", "validation"):
        selected = [row for row in revised if row["split"] == split]
        for suffix in ("", "-binary"):
            (out / "splits" / f"{split}{suffix}.jsonl").write_text(
                "".join(json.dumps(row, ensure_ascii=False) + "\n" for row in selected))
    schema["resolution_derivation"] = {
        "parent_dataset": str(root), "rgb": "unchanged native projection from source ERP",
        "road_targets": "new visually specified native 1024x512 polygons for every view",
        "semantic_nonroad_context": "retained parent partial context; not independently relabeled",
        "uncertain_boundary_radius_px": 3, "occlusion_policy": "ignore; no road filling",
        "annotation_status": annotations["annotation_status"],
        "independent_human_verification": False,
    }
    schema["annotation_revision"] = annotations["revision"]
    (out / "manifest.jsonl").write_text("".join(json.dumps(r, ensure_ascii=False) + "\n" for r in revised))
    write_json(out / "schema.json", schema)
    report["counts"] = dict(Counter(row["split"] for row in revised))
    report["finished_utc"] = datetime.now(timezone.utc).isoformat()
    write_json(out / "verification.json", report)
    print(json.dumps({"output": str(out), "counts": report["counts"],
                      "empty_road_views": sum(r["road_pixels"] == 0 for r in report["views"])}, ensure_ascii=False))


if __name__ == "__main__":
    main()

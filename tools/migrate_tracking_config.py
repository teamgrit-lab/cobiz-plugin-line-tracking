"""Migrate a deployment .env to the model-independent tracking configuration.

Preview by default. --write creates a private, exclusive backup before replacing
the file. Model selections, checkpoint pins and control values are preserved.
"""
import argparse
import os
from pathlib import Path
import re

OLD_PREFIX = "SWIN_L_"
RENAMED = {"ENGINE_DIR": "MODEL_DIR", "DEBUG_IMAGE": "IMAGE_NAME"}
REMOVED = {"ALLOW_BACKEND_FALLBACK", "PREPARED_CHECKPOINT_DIR", "TRT_ENGINE",
           "TRT_MANIFEST", "TRT_AUTO_BUILD", "TRT_CHECKPOINT", "TORCH_TENSORRT_VERSION",
           "STOP_ON_CAMERA_TIMESTAMP_INVALID", "STOP_ON_PATH_UNAVAILABLE",
           "STOP_ON_LOW_CONFIDENCE", "STOP_ON_UNSAFE_TIMEOUT", "STOP_ON_LATERAL_TARGET",
           "UNSAFE_TIMEOUT_SEC", "MAX_TARGET_HEADING_DEG"}
ASSIGNMENT = re.compile(r"^(\s*(?:export\s+)?)([A-Z][A-Z0-9_]*)(\s*=\s*)(.*)$")


def migrate(text):
    result = []
    keys = set()
    for line in text.splitlines():
        match = ASSIGNMENT.match(line)
        if match:
            lead, name, equal, value = match.groups()
            suffix = name.removeprefix(OLD_PREFIX).removeprefix("LINE_TRACKING_")
            if (name.startswith(OLD_PREFIX) or name.startswith("LINE_TRACKING_")) and suffix in REMOVED:
                continue
            if name.startswith(OLD_PREFIX):
                name = "LINE_TRACKING_" + RENAMED.get(suffix, suffix)
            if name in keys:
                raise ValueError(f"Duplicate destination setting: {name}; resolve it before migration")
            keys.add(name)
            if name in {"LINE_TRACKING_PROFILE", "LINE_TRACKING_BACKEND"}:
                selected = value.split(" #", 1)[0].strip().strip("\"'")
                allowed = ({"r50-fp16-640x360", "r50-finetuned-fp16-640x360"}
                           if name.endswith("PROFILE") else {"pytorch"})
                if selected not in allowed:
                    raise ValueError(f"Select a supported R50/PyTorch value for {name} before migration")
            line = lead + name + equal + value
        line = line.replace(OLD_PREFIX, "LINE_TRACKING_")
        line = line.replace("jetson-swin-l", "jetson-line-tracking")
        line = line.replace("cobiz-plugin-line-tracking-swin-l-debug", "cobiz-plugin-line-tracking-runtime")
        line = line.replace("/line_tracking/swin_l/", "/line_tracking/")
        line = line.replace("swin-l-tests", "line-tracking-tests")
        result.append(line)
    return "\n".join(result) + "\n"


def validate_environment(values):
    old = sorted(key for key in values if key.startswith(OLD_PREFIX))
    if old:
        raise ValueError("Retired tracking settings: " + ", ".join(old) +
                         "; run python3 tools/migrate_tracking_config.py --env-file .env --write")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--env-file", type=Path, default=Path(".env"))
    parser.add_argument("--write", action="store_true")
    args = parser.parse_args()
    original = args.env_file.read_text()
    try:
        updated = migrate(original)
        if not args.write:
            # Do not print deployment values, paths, or hashes to shared logs.
            print("Configuration can be migrated; use --write to apply with a private backup")
        elif updated == original:
            print("Configuration already uses current tracking settings")
        else:
            backup = args.env_file.with_name(args.env_file.name + ".before-line-tracking")
            fd = os.open(backup, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o600)
            with os.fdopen(fd, "w") as handle:
                handle.write(original)
            temporary = args.env_file.with_name(args.env_file.name + ".migrating")
            fd = os.open(temporary, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o600)
            with os.fdopen(fd, "w") as handle:
                handle.write(updated)
            temporary.replace(args.env_file)
            print(f"Migrated {args.env_file}; private backup: {backup}")
    except (ValueError, FileExistsError) as error:
        parser.error(str(error))


if __name__ == "__main__":
    main()

#!/bin/sh
set -eu
SIM_REPO=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
SIM_MODELS=${DANGJIN_360_MODEL_DIR:-"$SIM_REPO/.cache/dangjin-360-20261028"}
SIM_PYTHON=${DANGJIN_SIM_PYTHON:-"$SIM_REPO/.cache/connectome-venv/bin/python"}
SIM_ENCODER="$SIM_MODELS/flyvis_encoder.pt"
if [ ! -f "$SIM_ENCODER" ]; then SIM_ENCODER="$SIM_REPO/.cache/flyvis_encoder.pt"; fi
for SIM_FILE in "$SIM_MODELS/road/road-readout.pt" "$SIM_MODELS/policy/drive-readout.pt"; do
  if [ ! -f "$SIM_FILE" ]; then
    printf 'Missing 360 model: %s\nSee docs/dangjin-360.md for setup.\n' "$SIM_FILE" >&2
    exit 1
  fi
done
cd "$SIM_REPO"
export MPLBACKEND=Agg
exec "$SIM_PYTHON" tools/dangjin_sim_server.py --control neural --road-backend connectome \
  --bundle "$SIM_ENCODER" \
  --road-model "$SIM_MODELS/road" --policy-model "$SIM_MODELS/policy" \
  --device cpu --road-device mps --port 8775 "$@"

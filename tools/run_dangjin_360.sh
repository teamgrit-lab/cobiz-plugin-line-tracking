#!/bin/sh
set -eu
SIM_REPO=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
# Explicit legacy packages continue to use their original Flyvis runner.
SIM_CONTROL=neural
SIM_PORT=8775
if [ "${1:-}" = "--connectome-road" ]; then
  SIM_CONTROL=road-controller
  SIM_PORT=8780
  shift
elif [ "${1:-}" = "--connectome" ]; then
  shift
elif [ -z "${DANGJIN_360_MODEL_DIR:-}" ] || [ -n "${DANGJIN_FCN_MODEL_DIR:-}" ]; then
  SIM_MODEL=${DANGJIN_FCN_MODEL_DIR:-"/Volumes/MY_HDD/02_학습모델_데이터셋/03_FCN_ResNet18/모델/fcn1024-road-runs/20261103-081851-c90tq27l"}
  SIM_PYTHON=${DANGJIN_SIM_PYTHON:-"$SIM_REPO/.venv/bin/python"}
  if [ ! -f "$SIM_MODEL/complete.json" ]; then
    printf 'Set DANGJIN_FCN_MODEL_DIR to the fine-tuned FCN folder (complete.json and fcn_resnet18.onnx).\n' >&2
    exit 1
  fi
  exec "$SIM_PYTHON" "$SIM_REPO/tools/dangjin_sim_server.py" --fcn-model "$SIM_MODEL" --port 8775 "$@"
fi
SIM_MODELS=${DANGJIN_360_MODEL_DIR:-"$SIM_REPO/.cache/dangjin-360-20261028"}
SIM_PYTHON=${DANGJIN_SIM_PYTHON:-"$SIM_REPO/.cache/connectome-venv/bin/python"}
SIM_ENCODER="$SIM_MODELS/flyvis_encoder.pt"
if [ ! -f "$SIM_ENCODER" ]; then SIM_ENCODER="$SIM_REPO/.cache/flyvis_encoder.pt"; fi
for SIM_FILE in "$SIM_MODELS/road/road-readout.pt" "$SIM_ENCODER"; do
  if [ ! -f "$SIM_FILE" ]; then
    printf 'Missing 360 model: %s\nSee docs/dangjin-360.md for setup.\n' "$SIM_FILE" >&2
    exit 1
  fi
done
cd "$SIM_REPO"
export MPLBACKEND=Agg
if [ "$SIM_CONTROL" = neural ] && [ ! -f "$SIM_MODELS/policy/drive-readout.pt" ]; then
  printf 'Missing learned drive policy in %s/policy\n' "$SIM_MODELS" >&2
  exit 1
fi
exec "$SIM_PYTHON" tools/dangjin_sim_server.py --control "$SIM_CONTROL" --road-backend connectome \
  --bundle "$SIM_ENCODER" \
  --road-model "$SIM_MODELS/road" --policy-model "$SIM_MODELS/policy" \
  --device cpu --road-device mps --port "$SIM_PORT" "$@"

#!/bin/bash
set -euo pipefail

: "${ROS_DISTRO:?ROS_DISTRO must be set}"
# ROS/colcon setup scripts read several optional variables without defaults.
# Source them with nounset disabled, then restore the strict shell for the node.
set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "/unitree_ws/install/setup.bash"

TEAMGRIT_DDS_ENV="/opt/ros/teamgrit/dds/teamgrit_dds_env.sh"
if [[ ! -f "${TEAMGRIT_DDS_ENV}" ]]; then
  echo "[swin-l-debug] TeamGRIT DDS environment is required: ${TEAMGRIT_DDS_ENV}" >&2
  exit 1
fi
source "${TEAMGRIT_DDS_ENV}"
set -u

python3 - <<'PY'
import sys

try:
    import os
    import torch
    import torchvision
    import transformers
except ImportError as error:
    print(
        "[swin-l-debug] the base image must provide Jetson-compatible "
        "PyTorch, torchvision and Transformers dependencies: "
        f"{error}",
        file=sys.stderr,
    )
    raise SystemExit(1)

if os.environ.get("SWIN_L_BACKEND", "pytorch").lower() == "tensorrt":
    try:
        import tensorrt
    except ImportError as error:
        print(
            f"[swin-l-debug] TensorRT backend requested but unavailable: {error}",
            file=sys.stderr,
        )
        raise SystemExit(1)

print(
    f"[swin-l-debug] torch={torch.__version__} "
    f"torchvision={torchvision.__version__} "
    f"cuda_available={torch.cuda.is_available()} "
    f"transformers={transformers.__version__}",
    flush=True,
)
if "cuda" in os.environ.get("SWIN_L_DEVICE", "auto").lower() and not torch.cuda.is_available():
    print(
        "[swin-l-debug] SWIN_L_DEVICE requests CUDA but torch.cuda.is_available() is false",
        file=sys.stderr,
    )
    raise SystemExit(1)
PY

mode="${SWIN_L_MODE:-ros2}"
case "${mode}" in
  ros2) ;;
  task-drive) ;;
  *)
    echo "[swin-l-debug] unsupported SWIN_L_MODE: ${mode}" >&2
    exit 1
    ;;
esac

if [[ "${SWIN_L_BACKEND:-pytorch}" == "tensorrt" \
  && "${SWIN_L_TRT_AUTO_BUILD:-false}" == "true" ]]; then
  engine_path="${SWIN_L_TRT_ENGINE:?SWIN_L_TRT_ENGINE must be set}"
  manifest_path="${SWIN_L_TRT_MANIFEST:-${engine_path}.json}"
  checkpoint_path="${SWIN_L_TRT_CHECKPOINT:-/models/checkpoint}"
  engine_ready=false

  if [[ -s "${engine_path}" && -s "${manifest_path}" ]]; then
    echo "[swin-l-debug] validating existing TensorRT engine: ${engine_path}"
    if python3 /workspace/tools/validate_swin_l_tensorrt.py \
      --engine "${engine_path}" \
      --manifest "${manifest_path}"; then
      engine_ready=true
    else
      echo "[swin-l-debug] existing TensorRT artifact is invalid; rebuilding" >&2
    fi
  fi

  if [[ "${engine_ready}" != "true" ]]; then
    echo "[swin-l-debug] preparing TensorRT artifact before startup"
    mkdir -p \
      "$(dirname "${engine_path}")" \
      "$(dirname "${manifest_path}")" \
      "${checkpoint_path}"

    if [[ ! -s "${checkpoint_path}/model.safetensors" \
      || ! -s "${checkpoint_path}/checkpoint-manifest.json" ]]; then
      echo "[swin-l-debug] preparing Swin-L safetensors checkpoint at ${checkpoint_path}"
      python3 /workspace/tools/prepare_swin_l_checkpoint.py \
        --output-dir "${checkpoint_path}" \
        --allow-initialized-weights
    fi

    echo "[swin-l-debug] building TensorRT engine at ${engine_path}"
    python3 /workspace/tools/build_swin_l_tensorrt.py \
      --checkpoint "${checkpoint_path}" \
      --output "${engine_path}" \
      --manifest-output "${manifest_path}"
    python3 /workspace/tools/validate_swin_l_tensorrt.py \
      --engine "${engine_path}" \
      --manifest "${manifest_path}"
  fi
fi

exec python3 /workspace/tools/swin_l_local_path_debug.py "${mode}"

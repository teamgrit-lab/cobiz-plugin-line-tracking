#!/bin/bash
set -euo pipefail

# The camera-owned FCN service does not consume ROS or the Sidewalk backend.
if [[ "${LINE_TRACKING_MODE:-}" == "fcn360-detector" && " $* " == *" --direct-camera "* ]]; then
  exec python3 /workspace/tools/fcn360_detector.py "$@"
fi

# Reject an incompatible R50 backend before ROS setup or Swin-L engine builds.
if [[ ( "${LINE_TRACKING_PROFILE:-swin-l-aspect-224x384-fp16}" == "r50-fp16-640x360" \
  || "${LINE_TRACKING_PROFILE:-swin-l-aspect-224x384-fp16}" == "r50-finetuned-fp16-640x360" ) \
  && "${LINE_TRACKING_BACKEND:-pytorch}" != "pytorch" ]]; then
  echo "[line-tracking] R50 requires LINE_TRACKING_BACKEND=pytorch" >&2
  exit 1
fi

: "${ROS_DISTRO:?ROS_DISTRO must be set}"
# ROS/colcon setup scripts read several optional variables without defaults.
# Source them with nounset disabled, then restore the strict shell for the node.
set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "/unitree_ws/install/setup.bash"

TEAMGRIT_DDS_ENV="/opt/ros/teamgrit/dds/teamgrit_dds_env.sh"
if [[ ! -f "${TEAMGRIT_DDS_ENV}" ]]; then
  echo "[line-tracking] TeamGRIT DDS environment is required: ${TEAMGRIT_DDS_ENV}" >&2
  exit 1
fi
source "${TEAMGRIT_DDS_ENV}"
set -u

if [[ "${LINE_TRACKING_MODE:-}" == "fcn360-detector" ]]; then
  exec python3 /workspace/tools/fcn360_detector.py "$@"
fi
if [[ "${LINE_TRACKING_MODE:-}" == "connectome360-camera" ]]; then
  exec python3 /workspace/tools/connectome360_camera.py "$@"
fi

python3 - <<'PY'
import sys

try:
    import os
    import torch
    import torchvision
    import transformers
except ImportError as error:
    print(
        "[line-tracking] the base image must provide Jetson-compatible "
        "PyTorch, torchvision and Transformers dependencies: "
        f"{error}",
        file=sys.stderr,
    )
    raise SystemExit(1)

if os.environ.get("LINE_TRACKING_BACKEND", "pytorch").lower() == "tensorrt":
    try:
        import tensorrt
        import torch_tensorrt
    except ImportError as error:
        print(
            "[line-tracking] TensorRT hybrid backend requested but unavailable: "
            f"{error}",
            file=sys.stderr,
        )
        raise SystemExit(1)

print(
    f"[line-tracking] torch={torch.__version__} "
    f"torchvision={torchvision.__version__} "
    f"cuda_available={torch.cuda.is_available()} "
    f"transformers={transformers.__version__}",
    flush=True,
)
if "cuda" in os.environ.get("LINE_TRACKING_DEVICE", "auto").lower() and not torch.cuda.is_available():
    print(
        "[line-tracking] LINE_TRACKING_DEVICE requests CUDA but torch.cuda.is_available() is false",
        file=sys.stderr,
    )
    raise SystemExit(1)
PY

mode="${LINE_TRACKING_MODE:-ros2}"
case "${mode}" in
  ros2) ;;
  task-drive) ;;
  navigation-worker) ;;
  *)
    echo "[line-tracking] unsupported LINE_TRACKING_MODE: ${mode}" >&2
    exit 1
    ;;
esac

if [[ "${LINE_TRACKING_BACKEND:-pytorch}" == "tensorrt" \
  && "${LINE_TRACKING_TRT_AUTO_BUILD:-false}" == "true" ]]; then
  engine_path="${LINE_TRACKING_TRT_ENGINE:?LINE_TRACKING_TRT_ENGINE must be set}"
  manifest_path="${LINE_TRACKING_TRT_MANIFEST:-${engine_path}.json}"
  checkpoint_path="${LINE_TRACKING_TRT_CHECKPOINT:-/models/checkpoint}"
  engine_ready=false

  if [[ -s "${engine_path}" && -s "${manifest_path}" ]]; then
    echo "[line-tracking] validating existing TensorRT hybrid artifact: ${engine_path}"
    if python3 /workspace/tools/validate_swin_l_tensorrt.py \
      --engine "${engine_path}" \
      --manifest "${manifest_path}"; then
      engine_ready=true
    else
      echo "[line-tracking] existing TensorRT artifact is invalid; rebuilding" >&2
    fi
  fi

  if [[ "${engine_ready}" != "true" ]]; then
    echo "[line-tracking] preparing TensorRT artifact before startup"
    mkdir -p \
      "$(dirname "${engine_path}")" \
      "$(dirname "${manifest_path}")" \
      "${checkpoint_path}"

    if [[ ! -s "${checkpoint_path}/model.safetensors" \
      || ! -s "${checkpoint_path}/checkpoint-manifest.json" ]]; then
      echo "[line-tracking] preparing Swin-L safetensors checkpoint at ${checkpoint_path}"
      python3 /workspace/tools/prepare_swin_l_checkpoint.py \
        --output-dir "${checkpoint_path}" \
        --allow-initialized-weights
    fi

    echo "[line-tracking] building PyTorch/TensorRT hybrid artifact at ${engine_path}"
    python3 /workspace/tools/build_swin_l_tensorrt.py \
      --checkpoint "${checkpoint_path}" \
      --output "${engine_path}" \
      --manifest-output "${manifest_path}"
    python3 /workspace/tools/validate_swin_l_tensorrt.py \
      --engine "${engine_path}" \
      --manifest "${manifest_path}"
  fi
fi

exec python3 /workspace/tools/line_tracking_container.py "${mode}"

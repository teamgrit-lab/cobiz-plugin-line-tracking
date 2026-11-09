#!/bin/bash
set -euo pipefail

# The camera-owned FCN service does not consume ROS or the Sidewalk backend.
if [[ "${LINE_TRACKING_MODE:-}" == "fcn360-detector" && " $* " == *" --direct-camera "* ]]; then
  exec python3 /workspace/tools/fcn360_detector.py "$@"
fi

# Sidewalk R50 runs in PyTorch; the independent FCN service uses TensorRT.
if [[ "${LINE_TRACKING_BACKEND:-pytorch}" != "pytorch" ]]; then
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
  *)
    echo "[line-tracking] unsupported LINE_TRACKING_MODE: ${mode}" >&2
    exit 1
    ;;
esac

exec python3 /workspace/tools/line_tracking_container.py "${mode}"

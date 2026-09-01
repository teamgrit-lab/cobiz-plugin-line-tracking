#!/bin/bash
set -euo pipefail

: "${ROS_DISTRO:?ROS_DISTRO must be set}"
# ROS/colcon setup scripts read several optional variables without defaults.
# Source them with nounset disabled, then restore the strict shell for the node.
set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"

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

exec python3 /workspace/tools/swin_l_local_path_debug.py ros2

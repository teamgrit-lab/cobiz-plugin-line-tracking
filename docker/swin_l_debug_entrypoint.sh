#!/bin/bash
set -euo pipefail

: "${ROS_DISTRO:?ROS_DISTRO must be set}"
: "${SWIN_L_MODEL_PATH:?SWIN_L_MODEL_PATH must point to an exported ONNX model}"

set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"

TEAMGRIT_DDS_ENV="/opt/ros/teamgrit/dds/teamgrit_dds_env.sh"
if [[ ! -f "${TEAMGRIT_DDS_ENV}" ]]; then
  echo "[line-tracking-cpp] TeamGRIT DDS environment is required: ${TEAMGRIT_DDS_ENV}" >&2
  exit 1
fi
source "${TEAMGRIT_DDS_ENV}"
set -u

if [[ ! -r "${SWIN_L_MODEL_PATH}" ]]; then
  echo "[line-tracking-cpp] ONNX model is not readable: ${SWIN_L_MODEL_PATH}" >&2
  exit 1
fi

mode="${SWIN_L_MODE:-ros2}"
case "${mode}" in
  ros2|task-drive) ;;
  *)
    echo "[line-tracking-cpp] unsupported SWIN_L_MODE: ${mode}" >&2
    exit 1
    ;;
esac

exec /opt/cobiz-line-tracking/lib/cobiz_line_tracking/line_tracking_node

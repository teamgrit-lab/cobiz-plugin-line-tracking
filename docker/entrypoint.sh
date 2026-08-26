#!/bin/bash
set -euo pipefail

: "${ROS_DISTRO:?ROS_DISTRO must be set}"
source "/opt/ros/${ROS_DISTRO}/setup.bash"

TEAMGRIT_DDS_ENV="/opt/ros/teamgrit/dds/teamgrit_dds_env.sh"
if [[ ! -f "${TEAMGRIT_DDS_ENV}" ]]; then
  echo "[entrypoint] TeamGRIT DDS environment is required: ${TEAMGRIT_DDS_ENV}" >&2
  exit 1
fi
# shellcheck source=/dev/null
source "${TEAMGRIT_DDS_ENV}"
source /ros_ws/install/setup.bash

exec ros2 launch line_tracking line_tracking.launch.py

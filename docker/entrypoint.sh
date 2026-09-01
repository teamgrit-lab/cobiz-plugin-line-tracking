#!/bin/bash
set -euo pipefail

: "${ROS_DISTRO:?ROS_DISTRO must be set}"
# ROS/colcon setup scripts read several optional variables without defaults.
# Source them with nounset disabled, then restore the strict shell for the node.
set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"

TEAMGRIT_DDS_ENV="/opt/ros/teamgrit/dds/teamgrit_dds_env.sh"
if [[ ! -f "${TEAMGRIT_DDS_ENV}" ]]; then
  echo "[entrypoint] TeamGRIT DDS environment is required: ${TEAMGRIT_DDS_ENV}" >&2
  exit 1
fi
# shellcheck source=/dev/null
source "${TEAMGRIT_DDS_ENV}"
source /ros_ws/install/setup.bash
set -u

exec ros2 launch line_tracking line_tracking.launch.py

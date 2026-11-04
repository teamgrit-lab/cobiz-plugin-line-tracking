#!/bin/bash
set -eo pipefail
# The front camera originates on A2, so the inference process subscribes to its
# original ROS topic. USB 360 inference needs no ROS setup or camera publisher.
if [[ -n "${CONNECTUM_FRONT_CAMERA_TOPIC:-}" ]]; then
  source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
  source /unitree_ws/install/setup.bash
  source /opt/ros/teamgrit/dds/teamgrit_dds_env.sh
fi
exec /opt/connectome-venv/bin/python "$@"

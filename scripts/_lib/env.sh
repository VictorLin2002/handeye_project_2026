#!/usr/bin/env bash
# Canonical ROS 2 environment for this workspace.
set -euo pipefail

export WS_ROOT="${WS_ROOT:-/home/Victor/ros2_ws}"

# ROS setup scripts are not guaranteed to be nounset-safe.
set +u
source /opt/ros/jazzy/setup.bash
if [ -f "${WS_ROOT}/install/setup.bash" ]; then
  source "${WS_ROOT}/install/setup.bash"
fi
set -u

#!/usr/bin/env bash
# Run the hand-eye calibration GUI

set -euo pipefail

# Resolve workspace root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

# Source ROS2 environment (adjust if needed). Disable nounset while sourcing
# to avoid AMENT_TRACE_SETUP_FILES issues from set -u.
if [ -f "/opt/ros/humble/setup.bash" ]; then
    set +u
    source /opt/ros/humble/setup.bash
    set -u
elif [ -f "/opt/ros/jazzy/setup.bash" ]; then
    set +u
    source /opt/ros/jazzy/setup.bash
    set -u
fi

# Source workspace (disable nounset for colcon-generated setup scripts)
set +u
source "${WS_ROOT}/install/setup.bash"
set -u

# Run GUI
ros2 run handeye_gui handeye_gui

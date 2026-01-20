#!/usr/bin/env bash
# Start the tag localizer node (in the apriltag_detector package).
#
# This node integrates AprilTag detection, 3D reconstruction, and pose estimation.
# It subscribes to color and depth images from kinect_node and publishes object pose.
#
# Usage:
#   start_tag_localizer.sh                          # Use default config
#   start_tag_localizer.sh custom_params.yaml       # Use custom config
#   CONFIG=handeye.yaml start_tag_localizer.sh      # Use env variable
#
set -euo pipefail

# Ensure the workspace env (including ROS 2) is sourced before `ros2`.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/../_lib/env.sh"

# Get config file name from argument or environment variable
CONFIG_FILE="${1:-${CONFIG:-tag_localizer_params.yaml}}"

# Check if launch file exists
if ros2 pkg prefix apriltag_detector &>/dev/null; then
  # Use launch file with config (recommended)
  exec ros2 launch apriltag_detector tag_localizer.launch.py config:="${CONFIG_FILE}"
else
  echo "Warning: apriltag_detector not installed, using direct run (no config support)"
  exec ros2 run apriltag_detector tag_localizer_node
fi

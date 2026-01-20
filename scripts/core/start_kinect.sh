#!/usr/bin/env bash
# Start the Kinect ROS 2 node (image/depth source).
set -euo pipefail

# Set up the workspace environment so WS_ROOT is always defined.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/../_lib/env.sh"

cd "${WS_ROOT}"
exec ros2 run kinect kinect_node
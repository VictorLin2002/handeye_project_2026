#!/usr/bin/env bash
# Launch the Tag4 corner0 3D node with the workspace ROS environment.
set -euo pipefail

# scripts/handeye/run_tag4_cornor0.sh → ros2_ws
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/../_lib/env.sh"

exec ros2 run kinect tag4_corner0_3d_node "$@"

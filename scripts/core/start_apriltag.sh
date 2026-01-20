#!/usr/bin/env bash
# Start the AprilTag detector node.
set -euo pipefail

# Set up the workspace environment so WS_ROOT is always defined.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/../_lib/env.sh"

cd "${WS_ROOT}"
exec ros2 run apriltag_detector apriltag_node
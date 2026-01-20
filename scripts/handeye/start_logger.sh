#!/usr/bin/env bash
# Start the hand-eye data logger.
set -euo pipefail

# scripts/handeye/start_logger.sh → ros2_ws
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/../_lib/env.sh"

ROBOT_IP="${ROBOT_IP:-192.168.0.200}"
OBJECT_POSE_TOPIC="${OBJECT_POSE_TOPIC:-/apriltag/object_pose}"
CSV_PATH="${CSV_PATH:-${WS_ROOT}/handeye_samples.csv}"

exec ros2 run handeye_logger handeye_logger --ros-args   -p robot_ip:="${ROBOT_IP}"   -p object_pose_topic:="${OBJECT_POSE_TOPIC}"   -p csv_path:="${CSV_PATH}"

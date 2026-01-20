#!/usr/bin/env bash
# Run Tag4 corner0 touch verification using the bundled Python verifier.
#
# Usage:
#   ./scripts/handeye/verify.sh [-- <extra rclpy args>]
#
# Notes:
# - This script expects the core stack to be running (RTDE server + vision pipeline).
# - Parameters should be edited inside verify_tag4_simple.py, or overridden via ROS parameters.
set -euo pipefail

# scripts/handeye/verify.sh → ros2_ws
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "scripts/_lib/env.sh"

PY="${SCRIPT_DIR}/verify_tag4_simple.py"
if [ ! -f "${PY}" ]; then
  echo "Verifier script not found: ${PY}" >&2
  exit 1
fi

exec python3 "${PY}" "$@"

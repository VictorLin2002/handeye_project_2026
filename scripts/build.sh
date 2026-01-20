#!/usr/bin/env bash
# Build ROS 2 workspace using colcon (reproducible and environment-clean).
set -euo pipefail

# Resolve script directory.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Always start from a clean ROS/colcon environment to avoid polluted overlays.
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH \
      LD_LIBRARY_PATH PYTHONPATH

# Source the canonical environment (system ROS + workspace overlay).
source "${SCRIPT_DIR}/_lib/env.sh"

MODE="${1:-all}"

# Optional: allow passing extra colcon args after the mode.
shift || true
EXTRA_COLCON_ARGS=("$@")

cd "${WS_ROOT}"

# Common colcon arguments for consistent output.
COLCON_ARGS=(--symlink-install "${EXTRA_COLCON_ARGS[@]}")

case "${MODE}" in
  all)
    # Full clean rebuild.
    rm -rf build install log
    colcon build "${COLCON_ARGS[@]}"
    ;;
  vision)
    colcon build --packages-select kinect apriltag_detector "${COLCON_ARGS[@]}"
    ;;
  kinect)
    colcon build --packages-select kinect "${COLCON_ARGS[@]}"
    ;;
  apriltag)
    colcon build --packages-select apriltag_detector "${COLCON_ARGS[@]}"
    ;;
  handeye)
    colcon build --packages-select handeye_logger handeye_solver "${COLCON_ARGS[@]}"
    ;;
  *)
    echo "Usage: $0 {all|vision|kinect|apriltag|handeye} [-- <colcon args>...]" >&2
    echo "Example: $0 vision -- --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo" >&2
    exit 1
    ;;
esac

#!/usr/bin/env bash
# Manual operations menu for this workspace.
#
# This script is intended for human use only. It does not attempt to auto-diagnose
# the workspace state. Build and start the core stack using scripts/build.sh and
# scripts/start_all.sh as needed.
set -euo pipefail

# scripts/menu.sh → ros2_ws
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

show_menu() {
  cat <<'EOF'
========================
ROS 2 Workspace Menu
========================
1) Build: all
2) Build: vision (kinect + apriltag_detector)
3) Build: kinect
4) Build: apriltag_detector
5) Start hand-eye GUI
6) Start hand-eye logger
7) Run hand-eye solver
8) Generate calibration poses
9) Run calibration poses
10) Verify touch (handeye)
11) Start RTDE client (debug)
0) Exit
EOF
}

while true; do
  show_menu
  read -r -p "Select: " choice

  case "${choice}" in
    1)  "${SCRIPT_DIR}/build.sh" all ;;
    2)  "${SCRIPT_DIR}/build.sh" vision ;;
    3)  "${SCRIPT_DIR}/build.sh" kinect ;;
    4)  "${SCRIPT_DIR}/build.sh" apriltag ;;
    5)  ros2 run handeye_gui handeye_gui ;;
    6)  "${SCRIPT_DIR}/handeye/start_logger.sh" ;;
    7)  "${SCRIPT_DIR}/handeye/run_solver.sh" ;;
    8)  "${SCRIPT_DIR}/handeye/gen_calib_poses.sh" ;;
    9)  "${SCRIPT_DIR}/handeye/run_calib_poses.sh" ;;
    10) ros2 run handeye_verify verify_handeye_touch ;;
    11) "${SCRIPT_DIR}/tools/start_rtde_client.sh" ;;
    0)  exit 0 ;;
    *)  echo "Invalid selection." ;;
  esac
done

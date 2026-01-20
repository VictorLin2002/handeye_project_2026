#!/usr/bin/env bash
# Start the RTDE Action Server node (robot motion gateway).
#
# The RTDE server is the single entry point for motion commands.
# It should be started before any calibration or verification scripts.
set -euo pipefail

# Configuration (overridable via environment variables)
ROBOT_IP="${ROBOT_IP:-192.168.0.200}"

# Workspace safety constraints (meters)
WS_MIN_R="${WS_MIN_R:-0.20}"
WS_MAX_R="${WS_MAX_R:-0.8}"
WS_MIN_Z="${WS_MIN_Z:-0.20}"

exec ros2 run rtde_controller rtde_action_node --ros-args   -p robot_ip:="${ROBOT_IP}"   -p ws.min_r:="${WS_MIN_R}"   -p ws.max_r:="${WS_MAX_R}"   -p ws.min_z:="${WS_MIN_Z}"

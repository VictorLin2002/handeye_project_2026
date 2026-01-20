#!/usr/bin/env bash
# Start the core stack in one entry point:
# - RTDE Action Server
# - Kinect node
# - Tag localizer (with integrated AprilTag detection)
#
# Usage:
#   start_all.sh                      # Use default config
#   start_all.sh handeye_calib.yaml   # Use specific config for tag localizer
#   CONFIG=high_speed.yaml start_all.sh  # Use env variable
#
# Available configs:
#   - tag_localizer_params.yaml (default)
#   - handeye_calib.yaml (for hand-eye calibration)
#   - high_speed.yaml (fast detection)
#   - high_accuracy.yaml (precise tracking)
#
# Default: use tmux with one node per pane.
# Fallback: run as background processes and write logs.

set -euo pipefail

# Configuration for tag localizer
export CONFIG="${1:-${CONFIG:-tag_localizer_params.yaml}}"

# Resolve paths: scripts/start_all.sh -> ros2_ws/scripts
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CORE_DIR="${SCRIPT_DIR}/core"

# Prefer a stable log root under the workspace.
# If WS_ROOT is not set, infer it as scripts/.. (ros2_ws).
WS_ROOT="${WS_ROOT:-$(cd "${SCRIPT_DIR}/.." && pwd)}"
LOG_DIR="${WS_ROOT}/log/core"

mkdir -p "${LOG_DIR}"

# Build a command string executed inside each tmux pane.
# Notes:
# - Clear overlay-related variables to avoid polluted environments.
# - Source the canonical env.
# - Run the start script with logging.
pane_cmd() {
  local name="$1"
  local script="$2"
  local log="${LOG_DIR}/${name}.tmux.log"

  # Use a single bash -lc so we get predictable quoting and pipe status.
  # shellcheck disable=SC2016
  printf '%s' \
"bash -lc '
  unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH LD_LIBRARY_PATH PYTHONPATH
  source \"${SCRIPT_DIR}/_lib/env.sh\"
  \"${script}\" 2>&1 | tee -a \"${log}\"
  ec=\${PIPESTATUS[0]}
  echo \"[${name}] exited with code \${ec}\"
  echo \"log: ${log}\"
  exec bash
'"
}

run_in_pane() {
  local pane="$1"
  local name="$2"
  local script="$3"
  tmux send-keys -t "${pane}" "$(pane_cmd "${name}" "${script}")" C-m
}

# ----------------------------
# tmux mode
# ----------------------------
if command -v tmux >/dev/null 2>&1; then
  SESSION="ros2_core"

  if tmux has-session -t "${SESSION}" 2>/dev/null; then
    tmux kill-session -t "${SESSION}"
  fi

  tmux new-session -d -s "${SESSION}" -n core

  tmux split-window -h -t "${SESSION}:core"
  tmux split-window -v -t "${SESSION}:core.0"
  tmux select-layout -t "${SESSION}:core" tiled

  run_in_pane "${SESSION}:core.0" "rtde_server"   "${CORE_DIR}/start_rtde_server.sh"
  run_in_pane "${SESSION}:core.1" "kinect"        "${CORE_DIR}/start_kinect.sh"
  run_in_pane "${SESSION}:core.2" "tag_localizer" "${CORE_DIR}/start_tag_localizer.sh"

  echo ""
  echo "====================================="
  echo "Core stack started in tmux session: ${SESSION}"
  echo "Tag localizer config: ${CONFIG}"
  echo "====================================="
  echo ""

  tmux attach -t "${SESSION}"
  exit 0
fi

# ----------------------------
# fallback mode (no tmux)
# ----------------------------
pids=()

start_bg() {
  local name="$1"
  local script="$2"
  local log="${LOG_DIR}/${name}.log"

  # Ensure each process runs with the canonical env and a clean prefix set.
  # shellcheck disable=SC2016
  bash -lc "
    unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH LD_LIBRARY_PATH PYTHONPATH
    source \"${SCRIPT_DIR}/_lib/env.sh\"
    \"${script}\"
  " >"${log}" 2>&1 &

  pids+=("$!")
  echo "Started ${name} (pid=${pids[-1]}), log=${log}"
}

cleanup() {
  # Stop all background processes on exit or Ctrl-C.
  for pid in "${pids[@]:-}"; do
    if kill -0 "${pid}" 2>/dev/null; then
      kill "${pid}" 2>/dev/null || true
    fi
  done
}

trap cleanup EXIT INT TERM

start_bg "rtde_server"   "${CORE_DIR}/start_rtde_server.sh"
start_bg "kinect"        "${CORE_DIR}/start_kinect.sh"
start_bg "tag_localizer" "${CORE_DIR}/start_tag_localizer.sh"

echo ""
echo "====================================="
echo "Core stack started in background."
echo "Tag localizer config: ${CONFIG}"
echo "Logs directory: ${LOG_DIR}"
echo "PIDs: ${pids[*]}"
echo "====================================="
echo "Press Ctrl-C to stop all."
wait

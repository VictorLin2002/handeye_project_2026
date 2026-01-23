#!/usr/bin/env bash
# Execute calibration poses via RTDE action server (/move_to_pose) and trigger exactly one capture per pose.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/../_lib/env.sh"

CSV_PATH="${1:-${WS_ROOT}/calib_poses.csv}"
SPEED="${2:-0.80}"
ACC="${3:-0.30}"

LOGGER_STATUS_TOPIC="${LOGGER_STATUS_TOPIC:-/handeye_logger/status}"
CAPTURE_ACTION_NAME="${CAPTURE_ACTION_NAME:-/handeye_logger/capture_once}"

POSE_TIMEOUT_S="${POSE_TIMEOUT_S:-30}"
LOG_WAIT_TIMEOUT_S="${LOG_WAIT_TIMEOUT_S:-8}"

# Base settle after action done (increased to 1.5s to avoid motion blur)
SETTLE_S="${SETTLE_S:-1.5}"

# Extra settle if this pose differs a lot in rz (helps Z-rotation missing logs)
EXTRA_SETTLE_ON_BIG_RZ="${EXTRA_SETTLE_ON_BIG_RZ:-0.5}"
BIG_RZ_RAD="${BIG_RZ_RAD:-0.8}"   # ~45 deg, adjust as needed

POSE_PAUSE_S="${POSE_PAUSE_S:-0.2}"
ASK_ON_FAIL="${ASK_ON_FAIL:-0}"

# Polling rate for status waiting (Hz)
STATUS_POLL_HZ="${STATUS_POLL_HZ:-20}"

if [ ! -f "${CSV_PATH}" ]; then
  echo "CSV file not found: ${CSV_PATH}" >&2
  echo "Usage: $0 <calib_poses.csv> [speed] [acc]" >&2
  exit 1
fi

echo "Running calibration poses from: ${CSV_PATH}"
echo "Speed: ${SPEED} m/s, Acc: ${ACC} m/s^2"
echo "Logger status topic: ${LOGGER_STATUS_TOPIC}"
echo "Capture action: ${CAPTURE_ACTION_NAME}"
echo ""
echo "===== SETTLE TIMING CONFIG ====="
echo "Base settle (per pose):      ${SETTLE_S}s (avoid motion blur)"
echo "Extra settle (large rz):     ${EXTRA_SETTLE_ON_BIG_RZ}s"
echo "Pause between poses:         ${POSE_PAUSE_S}s"
echo "Tip: To reduce motion blur, increase SETTLE_S:"
echo "     export SETTLE_S=2.0 && $0 <csv>"
echo "================================"
echo ""

get_status() {
  # Output: "<count> <obj_stamp_us>" or "-1 -1"
  local line
  # Use stdbuf to prevent buffering issues and ensure timeout works
  line="$(timeout --signal=KILL 2.0 stdbuf -oL ros2 topic echo -n 1 "${LOGGER_STATUS_TOPIC}" 2>/dev/null || true)"
  if ! echo "${line}" | grep -q "data:"; then
    echo "-1 -1"
    return 0
  fi
  local count stamp
  count="$(echo "${line}" | sed -n 's/.*data: \[\([0-9]\+\),.*/\1/p' | head -n 1)"
  stamp="$(echo "${line}" | sed -n 's/.*data: \[[0-9]\+, \([0-9]\+\)\].*/\1/p' | head -n 1)"
  if [ -z "${count}" ] || [ -z "${stamp}" ]; then
    echo "-1 -1"
    return 0
  fi
  echo "${count} ${stamp}"
}

now_ms() {
  # Millisecond timestamp (bash-only, no python dependency)
  date +%s%3N
}

wait_logger_increment() {
  local before_count="$1"
  local before_stamp="$2"
  local t0_ms
  t0_ms="$(now_ms)"

  if [ "${before_count}" = "-1" ]; then
    return 2
  fi

  local sleep_s
  sleep_s="$(python3 - <<PY
hz=${STATUS_POLL_HZ}
print(f"{1.0/max(1.0,hz):.4f}")
PY
)"

  while true; do
    local now_count now_stamp
    read -r now_count now_stamp < <(get_status)
    if [ "${now_count}" = "-1" ]; then
      return 2
    fi

    if [ "${now_count}" -gt "${before_count}" ] && [ "${now_stamp}" != "${before_stamp}" ] && [ "${now_stamp}" -gt 0 ]; then
      return 0
    fi

    local t_ms
    t_ms="$(now_ms)"
    local elapsed_ms=$((t_ms - t0_ms))
    if [ "${elapsed_ms}" -ge $((LOG_WAIT_TIMEOUT_S * 1000)) ]; then
      return 1
    fi
    sleep "${sleep_s}"
  done
}

call_capture_once() {
  # Return 0 if action call succeeds, else 1.
  # Always prints action result for debugging.
  local out
  out="$(timeout --signal=KILL "${LOG_WAIT_TIMEOUT_S}" ros2 action send_goal "${CAPTURE_ACTION_NAME}" handeye_logger_interfaces/action/CaptureHandEye "{}" --feedback 2>&1 || true)"
  if [ -z "${out}" ]; then
    echo "Capture action: no response (timeout or call failed)"
    return 1
  fi

  # Extract and display the failure reason if present
  local msg
  msg="$(echo "${out}" | grep -oP "message: '\K[^']*" | tail -n 1 || true)"
  if [ -n "${msg}" ]; then
    echo "Capture result: ${msg}"
  else
    echo "Capture action response: ${out}"
  fi

  if echo "${out}" | grep -q "status: SUCCEEDED"; then
    return 0
  fi
  return 1
}

# Keep last rz to detect big Z rotation
LAST_RZ=""
line_idx=0

while IFS=',' read -r x y z rx ry rz; do
  line_idx=$((line_idx + 1))
  echo "Pose #${line_idx}: x=${x}, y=${y}, z=${z}, rx=${rx}, ry=${ry}, rz=${rz}"

  read -r before_count before_stamp < <(get_status)

  if ! timeout --signal=KILL "${POSE_TIMEOUT_S}" ros2 action send_goal /move_to_pose rtde_controller_interfaces/action/MoveToPose \
    "{x: ${x}, y: ${y}, z: ${z}, rx: ${rx}, ry: ${ry}, rz: ${rz}, speed: ${SPEED}, acc: ${ACC}}" \
    --feedback; then

    echo "Action failed for pose: (${x}, ${y}, ${z}), skipping to next..." >&2
    if [ "${ASK_ON_FAIL}" = "1" ]; then
      read -r -p "Continue? (y/n) " response
      if [ "${response}" != "y" ]; then
        exit 1
      fi
    fi
    # Continue to next pose
    LAST_RZ="${rz}"
    sleep "${POSE_PAUSE_S}"
    continue
  fi

  # Dynamic settle: add extra settle when rz changes a lot (helps Z-rotation tail settling)
  local_settle="${SETTLE_S}"
  if [ -n "${LAST_RZ}" ]; then
    big="$(python3 - <<PY
import math
rz=float("${rz}")
last=float("${LAST_RZ}")
thr=float("${BIG_RZ_RAD}")
d=abs(rz-last)
print("1" if d>thr else "0")
PY
)"
    if [ "${big}" = "1" ]; then
      local_settle="$(python3 - <<PY
base=float("${SETTLE_S}")
extra=float("${EXTRA_SETTLE_ON_BIG_RZ}")
print(f"{base+extra:.3f}")
PY
)"
      echo "Large rz change detected; using settle=${local_settle}s"
    fi
  fi

  sleep "${local_settle}"

  # Call capture_once action and print response
  cap_ok=0
  if ! call_capture_once; then
    cap_ok=1
    echo "WARN: capture_once action returned failure."
  fi

  # Even if capture action says success, still wait for status increment (ground truth)
  if wait_logger_increment "${before_count}" "${before_stamp}"; then
    echo "Logged OK (count increased)."
  else
    rc="$?"
    if [ "${rc}" -eq 1 ]; then
      echo "WARN: logger did not log within ${LOG_WAIT_TIMEOUT_S}s for this pose." >&2
    elif [ "${rc}" -eq 2 ]; then
      echo "WARN: logger status topic unavailable; cannot confirm logging." >&2
    fi

    if [ "${ASK_ON_FAIL}" = "1" ]; then
      read -r -p "Continue to next pose? (y/n) " response
      if [ "${response}" != "y" ]; then
        exit 1
      fi
    else
      echo "Continuing to next pose."
    fi
  fi

  LAST_RZ="${rz}"
  echo "---"
  sleep "${POSE_PAUSE_S}"
done < <(tail -n +2 "${CSV_PATH}")

echo ""
echo "Calibration sequence complete."

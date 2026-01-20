#!/usr/bin/env bash
# Run the hand-eye solver on a recorded dataset.
set -euo pipefail

# scripts/handeye/run_solver.sh → ros2_ws
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/../_lib/env.sh"

CSV_PATH="${1:-${WS_ROOT}/handeye_samples.csv}"
SOLVER_BIN="${WS_ROOT}/install/handeye_solver/lib/handeye_solver/handeye_solver"

if [ ! -f "${CSV_PATH}" ]; then
  echo "CSV file not found: ${CSV_PATH}" >&2
  exit 1
fi

exec "${SOLVER_BIN}" "${CSV_PATH}"

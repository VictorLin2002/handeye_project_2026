#!/usr/bin/env bash
# Start the RTDE move client (debug / manual use).
set -euo pipefail

# scripts/tools/start_rtde_client.sh → ros2_ws
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/../_lib/env.sh"

exec ros2 run rtde_controller move_client

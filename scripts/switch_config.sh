#!/usr/bin/env bash
# Quick config switcher for tag localizer
#
# Usage:
#   switch_config.sh                 # Interactive menu
#   switch_config.sh handeye         # Direct selection
#   switch_config.sh list            # List available configs

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="${WS_ROOT:-$(cd "${SCRIPT_DIR}/.." && pwd)}"
CONFIG_DIR="${WS_ROOT}/src/apriltag_detector/config"

# Color output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Available configurations
declare -A CONFIGS=(
    ["default"]="tag_localizer_params.yaml"
    ["handeye"]="handeye_calib.yaml"
    ["speed"]="high_speed.yaml"
    ["accuracy"]="high_accuracy.yaml"
)

declare -A DESCRIPTIONS=(
    ["default"]="Default balanced settings"
    ["handeye"]="Hand-eye calibration (no jump detection)"
    ["speed"]="High-speed detection (lower accuracy)"
    ["accuracy"]="High-accuracy tracking (lower speed)"
)

list_configs() {
    echo -e "${BLUE}Available configurations:${NC}"
    echo ""
    for key in "${!CONFIGS[@]}"; do
        local file="${CONFIGS[$key]}"
        local desc="${DESCRIPTIONS[$key]}"
        local path="${CONFIG_DIR}/${file}"

        if [ -f "$path" ]; then
            echo -e "  ${GREEN}✓${NC} ${YELLOW}${key}${NC} → ${file}"
            echo -e "    ${desc}"
        else
            echo -e "  ${RED}✗${NC} ${YELLOW}${key}${NC} → ${file} (NOT FOUND)"
        fi
        echo ""
    done
}

interactive_menu() {
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}  Tag Localizer Configuration Switcher${NC}"
    echo -e "${BLUE}========================================${NC}"
    echo ""

    local i=1
    local keys=()
    for key in default handeye speed accuracy; do
        keys+=("$key")
        echo -e "  ${YELLOW}${i})${NC} ${key} - ${DESCRIPTIONS[$key]}"
        ((i++))
    done

    echo ""
    read -p "Select configuration [1-4]: " choice

    case $choice in
        1) selected="default" ;;
        2) selected="handeye" ;;
        3) selected="speed" ;;
        4) selected="accuracy" ;;
        *)
            echo -e "${RED}Invalid choice${NC}"
            exit 1
            ;;
    esac

    echo "$selected"
}

# Main logic
case "${1:-interactive}" in
    list)
        list_configs
        exit 0
        ;;
    interactive)
        SELECTION=$(interactive_menu)
        ;;
    *)
        SELECTION="$1"
        ;;
esac

# Validate selection
if [ ! -v "CONFIGS[$SELECTION]" ]; then
    echo -e "${RED}Error: Unknown config '${SELECTION}'${NC}"
    echo ""
    list_configs
    exit 1
fi

CONFIG_FILE="${CONFIGS[$SELECTION]}"
CONFIG_PATH="${CONFIG_DIR}/${CONFIG_FILE}"

if [ ! -f "$CONFIG_PATH" ]; then
    echo -e "${RED}Error: Config file not found: ${CONFIG_PATH}${NC}"
    exit 1
fi

echo ""
echo -e "${GREEN}Selected configuration: ${SELECTION}${NC}"
echo -e "File: ${CONFIG_FILE}"
echo -e "Description: ${DESCRIPTIONS[$SELECTION]}"
echo ""
echo -e "${YELLOW}To use this configuration:${NC}"
echo ""
echo -e "  ${BLUE}# Option 1: Start with specific config${NC}"
echo -e "  ./scripts/start_all.sh ${CONFIG_FILE}"
echo ""
echo -e "  ${BLUE}# Option 2: Set environment variable${NC}"
echo -e "  export CONFIG=${CONFIG_FILE}"
echo -e "  ./scripts/start_all.sh"
echo ""
echo -e "  ${BLUE}# Option 3: Start tag localizer only${NC}"
echo -e "  ./scripts/core/start_tag_localizer.sh ${CONFIG_FILE}"
echo ""

# Offer to view the config
read -p "View configuration file? [y/N]: " view_choice
if [[ "$view_choice" =~ ^[Yy]$ ]]; then
    echo ""
    echo -e "${BLUE}========== ${CONFIG_FILE} ==========${NC}"
    cat "$CONFIG_PATH"
    echo -e "${BLUE}======================================${NC}"
fi

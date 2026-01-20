#!/bin/bash
# Quick script to view calibration results

RESULTS_DIR="/home/Victor/ros2_ws/results/handeye_calibration"

# Colors
BLUE='\033[0;34m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}Hand-Eye Calibration Results Viewer${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# Check if results directory exists
if [ ! -d "$RESULTS_DIR" ]; then
    echo -e "${YELLOW}No results directory found.${NC}"
    echo "Run a test first: scripts/handeye/verify/run_repeatability_test.sh"
    exit 1
fi

# List available results
echo -e "${GREEN}Available Results:${NC}"
echo ""

DATA_COUNT=$(ls -1 "$RESULTS_DIR/data"/*.csv 2>/dev/null | wc -l)
PLOT_COUNT=$(ls -1 "$RESULTS_DIR/plots"/*.png 2>/dev/null | wc -l)
REPORT_COUNT=$(ls -1 "$RESULTS_DIR/reports"/*.md 2>/dev/null | wc -l)

echo "  Data files:   $DATA_COUNT"
echo "  Plots:        $PLOT_COUNT"
echo "  Reports:      $REPORT_COUNT"
echo ""

if [ $DATA_COUNT -eq 0 ]; then
    echo -e "${YELLOW}No test results found yet.${NC}"
    exit 0
fi

# Show latest results
echo -e "${GREEN}Latest Results:${NC}"
echo ""

LATEST_DATA=$(ls -t "$RESULTS_DIR/data"/*.csv 2>/dev/null | head -1)
LATEST_PLOT=$(ls -t "$RESULTS_DIR/plots"/*.png 2>/dev/null | head -1)
LATEST_REPORT=$(ls -t "$RESULTS_DIR/reports"/*.md 2>/dev/null | head -1)

if [ -n "$LATEST_DATA" ]; then
    echo "  Data:   $(basename "$LATEST_DATA")"
fi
if [ -n "$LATEST_PLOT" ]; then
    echo "  Plot:   $(basename "$LATEST_PLOT")"
fi
if [ -n "$LATEST_REPORT" ]; then
    echo "  Report: $(basename "$LATEST_REPORT")"
fi

echo ""
echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}What would you like to do?${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""
echo "1) View latest plot"
echo "2) View latest report"
echo "3) List all results"
echo "4) Open results directory"
echo "5) Exit"
echo ""
read -p "Enter choice [1-5]: " choice

case $choice in
    1)
        if [ -n "$LATEST_PLOT" ]; then
            echo "Opening plot..."
            xdg-open "$LATEST_PLOT" 2>/dev/null || echo "Could not open plot. File: $LATEST_PLOT"
        else
            echo "No plots found."
        fi
        ;;
    2)
        if [ -n "$LATEST_REPORT" ]; then
            echo ""
            cat "$LATEST_REPORT"
        else
            echo "No reports found."
        fi
        ;;
    3)
        echo ""
        echo -e "${GREEN}All Data Files:${NC}"
        ls -lht "$RESULTS_DIR/data"/*.csv 2>/dev/null | head -10
        echo ""
        echo -e "${GREEN}All Plots:${NC}"
        ls -lht "$RESULTS_DIR/plots"/*.png 2>/dev/null | head -10
        echo ""
        echo -e "${GREEN}All Reports:${NC}"
        ls -lht "$RESULTS_DIR/reports"/*.md 2>/dev/null | head -10
        ;;
    4)
        echo "Opening results directory..."
        xdg-open "$RESULTS_DIR" 2>/dev/null || nautilus "$RESULTS_DIR" 2>/dev/null || echo "Results directory: $RESULTS_DIR"
        ;;
    5)
        echo "Goodbye!"
        exit 0
        ;;
    *)
        echo "Invalid choice."
        exit 1
        ;;
esac

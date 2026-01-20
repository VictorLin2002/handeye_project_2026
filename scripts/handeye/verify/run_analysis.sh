#!/bin/bash
# Quick analysis script for hand-eye calibration repeatability

# Default values
CSV_FILE=""
REFERENCE="0.0 -650.0 83.5"
OUTPUT="/home/Victor/ros2_ws/results/handeye_calibration/plots/calibration_analysis_$(date +%Y%m%d_%H%M%S).png"
MODE="full"

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --csv)
            CSV_FILE="$2"
            shift 2
            ;;
        --reference)
            REFERENCE="$2 $3 $4"
            shift 4
            ;;
        --output)
            OUTPUT="$2"
            shift 2
            ;;
        --simple)
            MODE="simple"
            shift
            ;;
        --help)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --csv FILE           Path to CSV file with sample data"
            echo "  --reference X Y Z    Reference point in mm (default: 0.0 -650.0 83.5)"
            echo "  --output FILE        Output plot filename (default: results/handeye_calibration/plots/...)"
            echo "  --simple             Use simple 2-panel plot instead of 6-panel"
            echo "  --help               Show this help message"
            echo ""
            echo "Examples:"
            echo "  $0 --csv /tmp/samples.csv"
            echo "  $0 --csv /tmp/samples.csv --simple"
            echo "  $0 --csv /tmp/samples.csv --reference 0 -650 83.5 --output result.png"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
done

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Build command
CMD="python3 $SCRIPT_DIR/plot_repeatability_errors.py --reference $REFERENCE --output $OUTPUT"

if [ -n "$CSV_FILE" ]; then
    CMD="$CMD --csv $CSV_FILE"
fi

if [ "$MODE" = "simple" ]; then
    CMD="$CMD --simple"
fi

# Run
echo "========================================"
echo "Hand-Eye Calibration Error Analysis"
echo "========================================"
echo "CSV file:   ${CSV_FILE:-[simulated data]}"
echo "Reference:  $REFERENCE mm"
echo "Output:     $OUTPUT"
echo "Mode:       $MODE"
echo "========================================"
echo ""

$CMD

if [ $? -eq 0 ]; then
    echo ""
    echo "✓ Analysis complete!"
    echo "✓ Plot saved to: $OUTPUT"

    # Try to open the image
    if command -v xdg-open &> /dev/null; then
        echo ""
        read -p "Open plot now? (y/n) " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            xdg-open "$OUTPUT" &
        fi
    fi
else
    echo ""
    echo "✗ Analysis failed!"
    exit 1
fi

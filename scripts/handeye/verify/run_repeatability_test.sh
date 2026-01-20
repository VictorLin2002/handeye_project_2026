#!/bin/bash
# Complete workflow: Run repeatability test and generate analysis plots

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Default parameters
NUM_SAMPLES=50
SAMPLE_INTERVAL=0.5
REFERENCE_X=0.0
REFERENCE_Y=-650.0
REFERENCE_Z=83.5
OUTPUT_DIR="/home/Victor/ros2_ws/results/handeye_calibration"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
CSV_FILE="${OUTPUT_DIR}/calibration_samples_${TIMESTAMP}.csv"
PLOT_FILE="${OUTPUT_DIR}/calibration_analysis_${TIMESTAMP}.png"
PLOT_MODE="full"

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --samples)
            NUM_SAMPLES="$2"
            shift 2
            ;;
        --interval)
            SAMPLE_INTERVAL="$2"
            shift 2
            ;;
        --reference)
            REFERENCE_X="$2"
            REFERENCE_Y="$3"
            REFERENCE_Z="$4"
            shift 4
            ;;
        --output-dir)
            OUTPUT_DIR="$2"
            # Create directory structure if it doesn't exist
            mkdir -p "$OUTPUT_DIR"
            shift 2
            ;;
        --simple)
            PLOT_MODE="simple"
            shift
            ;;
        --help)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Run hand-eye calibration repeatability test and generate analysis plots"
            echo ""
            echo "Options:"
            echo "  --samples N          Number of samples to collect (default: 50)"
            echo "  --interval SEC       Interval between samples in seconds (default: 0.5)"
            echo "  --reference X Y Z    Reference point in mm (default: 0.0 -650.0 83.5)"
            echo "  --output-dir DIR     Output directory for results (default: ~/ros2_ws/results/handeye_calibration)"
            echo "  --simple             Use simple 2-panel plot (default: 6-panel)"
            echo "  --help               Show this help message"
            echo ""
            echo "Prerequisites:"
            echo "  - ROS2 workspace sourced"
            echo "  - AprilTag detector running"
            echo "  - Camera publishing tag corners"
            echo ""
            echo "Example:"
            echo "  $0 --samples 100 --interval 0.3"
            echo "  $0 --reference 0 -650 85 --simple"
            exit 0
            ;;
        *)
            echo -e "${RED}Unknown option: $1${NC}"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
done

# Ensure output directory exists
mkdir -p "${OUTPUT_DIR}"

# Update file paths with actual output dir
CSV_FILE="${OUTPUT_DIR}/data/calibration_samples_${TIMESTAMP}.csv"
PLOT_FILE="${OUTPUT_DIR}/plots/calibration_analysis_${TIMESTAMP}.png"

# Create subdirectories
mkdir -p "${OUTPUT_DIR}/data"
mkdir -p "${OUTPUT_DIR}/plots"
mkdir -p "${OUTPUT_DIR}/reports"

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}Hand-Eye Calibration Repeatability Test${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""
echo -e "${YELLOW}Configuration:${NC}"
echo "  Samples:       $NUM_SAMPLES"
echo "  Interval:      ${SAMPLE_INTERVAL}s"
echo "  Reference:     ($REFERENCE_X, $REFERENCE_Y, $REFERENCE_Z) mm"
echo "  Output Dir:    $OUTPUT_DIR"
echo "  CSV File:      $CSV_FILE"
echo "  Plot File:     $PLOT_FILE"
echo "  Plot Mode:     $PLOT_MODE"
echo ""

# Check if ROS2 is sourced
if ! command -v ros2 &> /dev/null; then
    echo -e "${RED}✗ Error: ros2 command not found${NC}"
    echo "Please source your ROS2 workspace:"
    echo "  source /home/Victor/ros2_ws/install/setup.bash"
    exit 1
fi

# Check if topics are available
echo -e "${YELLOW}Checking required topics...${NC}"
REQUIRED_TOPICS=(
    "/apriltag/tag4_corner0_3d"
    "/apriltag/tag4_corner1_3d"
    "/apriltag/tag4_corner2_3d"
    "/apriltag/tag4_corner3_3d"
)

MISSING_TOPICS=0
for topic in "${REQUIRED_TOPICS[@]}"; do
    if ros2 topic list 2>/dev/null | grep -q "^${topic}$"; then
        echo -e "  ${GREEN}✓${NC} $topic"
    else
        echo -e "  ${RED}✗${NC} $topic (not available)"
        MISSING_TOPICS=$((MISSING_TOPICS + 1))
    fi
done

if [ $MISSING_TOPICS -gt 0 ]; then
    echo ""
    echo -e "${YELLOW}Warning: $MISSING_TOPICS topic(s) not available${NC}"
    echo "Make sure AprilTag detector is running and detecting tag4"
    echo ""
    read -p "Continue anyway? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "Aborted."
        exit 1
    fi
fi

echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Step 1/3: Running Repeatability Test${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo "This will collect $NUM_SAMPLES samples..."
echo "Press Ctrl+C to abort if needed"
echo ""

# Run the repeatability test
python3 "$SCRIPT_DIR/verify_tag4_simple.py" \
    --ros-args \
    -p test_mode:=repeatability \
    -p num_samples:=${NUM_SAMPLES} \
    -p sample_interval_sec:=${SAMPLE_INTERVAL} \
    -p reference_point_mm:="[${REFERENCE_X}, ${REFERENCE_Y}, ${REFERENCE_Z}]" \
    -p save_samples_csv:="${CSV_FILE}"

TEST_RESULT=$?

if [ $TEST_RESULT -ne 0 ]; then
    echo ""
    echo -e "${RED}✗ Test failed or was interrupted${NC}"
    exit 1
fi

# Check if CSV was created
if [ ! -f "$CSV_FILE" ]; then
    echo ""
    echo -e "${RED}✗ CSV file was not created: $CSV_FILE${NC}"
    exit 1
fi

# Count samples in CSV
SAMPLE_COUNT=$(tail -n +2 "$CSV_FILE" | wc -l)
echo ""
echo -e "${GREEN}✓ Test complete! Collected $SAMPLE_COUNT samples${NC}"
echo -e "${GREEN}✓ Data saved to: $CSV_FILE${NC}"

echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Step 2/3: Generating Analysis Plots${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""

# Generate plots
PLOT_CMD="python3 $SCRIPT_DIR/plot_repeatability_errors.py \
    --csv $CSV_FILE \
    --reference $REFERENCE_X $REFERENCE_Y $REFERENCE_Z \
    --output $PLOT_FILE"

if [ "$PLOT_MODE" = "simple" ]; then
    PLOT_CMD="$PLOT_CMD --simple"
fi

$PLOT_CMD

PLOT_RESULT=$?

if [ $PLOT_RESULT -ne 0 ]; then
    echo ""
    echo -e "${RED}✗ Plot generation failed${NC}"
    exit 1
fi

echo ""
echo -e "${GREEN}✓ Plot generated: $PLOT_FILE${NC}"

echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Step 3/3: Results Summary${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""

# Display file sizes
CSV_SIZE=$(du -h "$CSV_FILE" | cut -f1)
PLOT_SIZE=$(du -h "$PLOT_FILE" | cut -f1)

echo -e "${BLUE}Output Files:${NC}"
echo "  CSV:   $CSV_FILE ($CSV_SIZE)"
echo "  Plot:  $PLOT_FILE ($PLOT_SIZE)"
echo ""

# Extract statistics from CSV
echo -e "${BLUE}Quick Statistics:${NC}"
python3 - <<EOF
import csv
import numpy as np

with open('$CSV_FILE', 'r') as f:
    reader = csv.DictReader(f)
    samples = []
    for row in reader:
        samples.append([float(row['x_mm']), float(row['y_mm']), float(row['z_mm'])])

samples = np.array(samples)
ref = np.array([$REFERENCE_X, $REFERENCE_Y, $REFERENCE_Z])

mean = samples.mean(axis=0)
std = samples.std(axis=0, ddof=1)
bias = mean - ref
norm_bias = np.linalg.norm(bias)

print(f"  Mean (mm):  X={mean[0]:+7.2f}, Y={mean[1]:+7.2f}, Z={mean[2]:+7.2f}")
print(f"  Bias (mm):  X={bias[0]:+7.2f}, Y={bias[1]:+7.2f}, Z={bias[2]:+7.2f}")
print(f"  Std  (mm):  X={std[0]:7.2f}, Y={std[1]:7.2f}, Z={std[2]:7.2f}")
print(f"  Norm Bias:  {norm_bias:.2f} mm")
print()

# Quality assessment
if norm_bias < 5:
    quality = "✓ Excellent"
    color = "\033[0;32m"
elif norm_bias < 10:
    quality = "⚠ Acceptable"
    color = "\033[1;33m"
else:
    quality = "✗ Needs Recalibration"
    color = "\033[0;31m"

print(f"{color}  Quality: {quality}\033[0m")
EOF

echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}✓ Analysis Complete!${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""

# Ask if user wants to open the plot
if command -v xdg-open &> /dev/null; then
    read -p "Open plot now? (y/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        xdg-open "$PLOT_FILE" &
        echo "Plot opened in default viewer"
    fi
fi

echo ""
echo "Files saved to:"
echo "  $CSV_FILE"
echo "  $PLOT_FILE"
echo ""
echo "To re-generate plots with different settings:"
echo "  $SCRIPT_DIR/run_analysis.sh --csv $CSV_FILE"
echo ""

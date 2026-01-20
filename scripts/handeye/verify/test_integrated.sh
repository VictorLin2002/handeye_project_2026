#!/bin/bash
# Quick test script for integrated plotting functionality

echo "========================================="
echo "Testing Integrated Plotting in verify_tag4_simple.py"
echo "========================================="
echo ""

cd /home/Victor/ros2_ws

echo "Test 1: Default behavior (auto_plot=True, full mode)"
echo "----------------------------------------------"
python3 scripts/handeye/verify/verify_tag4_simple.py \
  --ros-args \
  -p test_mode:=repeatability \
  -p num_samples:=10 \
  -p sample_interval_sec:=0.2

echo ""
echo "========================================="
echo "Check results:"
echo "  CSV:   results/handeye_calibration/data/"
echo "  Plot:  results/handeye_calibration/plots/"
echo "========================================="

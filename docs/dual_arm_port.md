# Hand-Eye Calibration — Dual-Arm Port

This project has been ported to `vmrl-robot/vmrl-delta-dual-arm-robot`
(branch `claude/port-handeye-dual-arm-bbUls`).

## What Changed

The original `handeye_project_2026` used a **UR robot via RTDE**.  
The dual-arm port replaces all RTDE calls with ROS 2 topics/services published
by the arm driver.

| Component | Original | Dual-Arm Port |
|-----------|----------|---------------|
| Robot motion | `MoveToPose` action → RTDE | `dual_arm_move_server` bridges `MoveToPose` → `/{arm}/MoveL_ik` service |
| TCP pose | `RTDEReceiveInterface.getActualTCPPose()` at 500 Hz | Subscribe `/{arm}/tcp_pose` (PoseStamped) at ~20 Hz |
| Camera | Azure Kinect (K4A SDK) | Same Kinect via `camera_driver` node |
| AprilTag detection | `tag_localizer_node.cpp` (full PnP + depth) | `apriltag_pose_bridge` (PnP from pixel corners + camera_info) |
| Solver | `handeye_solver` (C++ binary) | Same binary, identical CSV format |

## New Packages in Dual-Arm Repo

```
src/handeye/
├── rtde_controller_interfaces/   # MoveToPose.action definition
├── handeye_logger_interfaces/    # CaptureHandEye.action definition
├── dual_arm_move_server/         # /move_to_pose -> /right_arm/MoveL_ik
├── handeye_logger/               # data logger (tcp_pose topic edition)
├── handeye_solver/               # C++ AX=XB solver (unchanged)
└── apriltag_pose_bridge/         # detections -> /apriltag/object_pose
```

## Calibration Workflow

```bash
# 1. Start the dual-arm system (arm driver + movel_ik + camera + apriltag)
./start_dual_arm_system.sh

# 2. Start hand-eye calibration components
ARM_PREFIX=right_arm CAMERA_NAME=camera_L TAG_ID=0 TAG_SIZE_M=0.12 \
  ./scripts/handeye/start_handeye.sh

# 3. Edit CAMERA_POS / ORIGIN / RC_RV in gen_calib_poses.sh (one-time setup)
vim scripts/handeye/gen_calib_poses.sh

# 4. Generate calibration pose grid (~80 poses)
./scripts/handeye/gen_calib_poses.sh

# 5. Execute poses and collect samples (~10 min)
./scripts/handeye/run_calib_poses.sh

# 6. Solve for T_Base_Camera
./scripts/handeye/run_solver.sh
```

## Build

```bash
cd ~/yucheng_ws
colcon build --packages-select \
  rtde_controller_interfaces \
  handeye_logger_interfaces \
  dual_arm_move_server \
  handeye_logger \
  handeye_solver \
  apriltag_pose_bridge
source install/setup.bash
```

## Parameter Tuning

### handeye_logger
| Parameter | Default | Notes |
|-----------|---------|-------|
| `stationary_window` | 20 | Samples needed for stationarity (20 × 50ms ≈ 1 s) |
| `pos_eps_m` | 0.0001 | Position variation threshold (0.1 mm) |
| `rot_eps_rad` | 0.003 | Rotation variation threshold (~0.17°) |
| `max_sync_dt_s` | 0.10 | Max time difference between TCP and vision |
| `max_reprojection_error_px` | 2.0 | Vision quality gate |

### gen_calib_poses.sh
| Variable | Default | Notes |
|----------|---------|-------|
| `CAMERA_POS` | placeholder | **Must be set to actual Kinect position in Base frame** |
| `ORIGIN` | placeholder | Calibration grid centre in Base frame |
| `RC_RV` | placeholder | TCP rotvec when tag faces camera squarely |

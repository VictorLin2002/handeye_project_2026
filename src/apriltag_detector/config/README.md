# AprilTag Localizer Configuration Guide

This directory contains configuration files for the `tag_localizer_node`.

## Available Configurations

### 1. `tag_localizer_params.yaml` (Default)
**Use case:** General-purpose tracking with balanced speed and accuracy

**Key settings:**
- Full resolution detection (quad_decimate: 1.0)
- Edge refinement enabled
- RMSE threshold: 50mm
- No jump detection (suitable for various scenarios)
- No smoothing (alpha: 1.0)

**When to use:**
- General development
- Testing
- When you don't have specific requirements

---

### 2. `handeye_calib.yaml`
**Use case:** Hand-eye calibration data collection

**Key settings:**
- Full resolution with edge refinement
- **Jump detection DISABLED** (critical for manual movement)
- Relaxed RMSE threshold: 100mm
- No smoothing for raw measurements

**When to use:**
- Running hand-eye calibration scripts
- Collecting calibration poses
- When manually moving the robot or object

**Important:** This is the **ONLY** configuration that should be used during hand-eye calibration!

---

### 3. `high_speed.yaml`
**Use case:** Fast detection at the cost of some accuracy

**Key settings:**
- Half resolution (quad_decimate: 2.0) → 4x faster
- Edge refinement DISABLED
- 8 threads for maximum parallelism
- Smaller depth window (5x5)
- Relaxed RMSE: 80mm

**When to use:**
- Real-time applications
- When frame rate is critical
- Dynamic object tracking
- Testing/debugging

**Performance:** ~60 FPS on modern CPUs (vs ~30 FPS default)

---

### 4. `high_accuracy.yaml`
**Use case:** Maximum precision tracking

**Key settings:**
- Full resolution with edge refinement
- Large depth window (11x11)
- Strict RMSE threshold: 20mm
- Strict jump detection (20mm position, 3° rotation)
- Heavy smoothing (alpha: 0.15)

**When to use:**
- Static object tracking
- Precision measurements
- Quality assurance
- When speed is not critical

**Performance:** ~15-20 FPS (slower due to large depth window)

---

## How to Use

### Method 1: Using `start_all.sh`

```bash
# Use default config
./scripts/start_all.sh

# Use specific config
./scripts/start_all.sh handeye_calib.yaml
./scripts/start_all.sh high_speed.yaml

# Use environment variable
export CONFIG=high_accuracy.yaml
./scripts/start_all.sh
```

### Method 2: Using `start_tag_localizer.sh` directly

```bash
# Default config
./scripts/core/start_tag_localizer.sh

# Specific config
./scripts/core/start_tag_localizer.sh handeye_calib.yaml

# Environment variable
CONFIG=high_speed.yaml ./scripts/core/start_tag_localizer.sh
```

### Method 3: Using launch file directly

```bash
# Default
ros2 launch apriltag_detector tag_localizer.launch.py

# Custom config
ros2 launch apriltag_detector tag_localizer.launch.py config:=handeye_calib.yaml
```

### Method 4: Interactive config switcher

```bash
# Interactive menu
./scripts/switch_config.sh

# Direct selection
./scripts/switch_config.sh handeye
./scripts/switch_config.sh speed
./scripts/switch_config.sh accuracy

# List available configs
./scripts/switch_config.sh list
```

---

## Creating Custom Configurations

1. Copy an existing config file:
   ```bash
   cd /home/Victor/ros2_ws/src/apriltag_detector/config
   cp tag_localizer_params.yaml my_custom.yaml
   ```

2. Edit parameters:
   ```bash
   nano my_custom.yaml
   ```

3. Use your custom config:
   ```bash
   ./scripts/start_all.sh my_custom.yaml
   ```

---

## Parameter Reference

### AprilTag Detection

| Parameter | Type | Description | Impact |
|-----------|------|-------------|--------|
| `quad_decimate` | double | Image decimation (1.0=full, 2.0=half) | Higher = faster but less accurate |
| `quad_sigma` | double | Gaussian blur (0.0-1.0) | Noise reduction |
| `nthreads` | int | Parallel threads | More = faster (diminishing returns) |
| `refine_edges` | bool | Sub-pixel refinement | True = slower but more accurate |

### Depth Sampling

| Parameter | Type | Description | Impact |
|-----------|------|-------------|--------|
| `depth_window_radius` | int | Window size (r=3 → 7x7) | Larger = more robust but slower |
| `depth_min_valid_count` | int | Minimum valid samples | Higher = more strict filtering |
| `depth_outlier_thresh_mm` | int | Outlier trimming (0=disabled) | Removes noise spikes |

### Pose Filtering

| Parameter | Type | Description | Impact |
|-----------|------|-------------|--------|
| `enable_pose_filtering` | bool | Enable RMSE filtering | Should always be true |
| `max_rmse_threshold_mm` | double | Maximum allowed RMSE | Lower = stricter quality |
| `enable_jump_detection` | bool | Enable temporal consistency | **FALSE for hand-eye calib!** |
| `max_position_jump_m` | double | Max position change per frame | Lower = reject fast movement |
| `max_rotation_jump_deg` | double | Max rotation change per frame | Lower = reject fast rotation |
| `smoothing_alpha` | double | Smoothing factor (0.0-1.0) | 1.0=none, 0.0=heavy |

---

## Troubleshooting

### Problem: "Pose REJECTED: RMSE too high"
**Solution:** Increase `max_rmse_threshold_mm` or check depth quality

### Problem: "Position jump too large"
**Solution:**
- For hand-eye calib: Use `handeye_calib.yaml`
- For tracking: Increase `max_position_jump_m`

### Problem: Low frame rate
**Solution:** Use `high_speed.yaml` or increase `quad_decimate`

### Problem: Jittery pose output
**Solution:**
- Decrease `smoothing_alpha` (e.g., 0.3)
- Increase `depth_window_radius`

### Problem: Config not loaded
**Solution:**
- Rebuild: `colcon build --packages-select apriltag_detector`
- Source: `source install/setup.bash`

---

## Quick Reference Table

| Use Case | Recommended Config | Key Feature |
|----------|-------------------|-------------|
| Hand-Eye Calibration | `handeye_calib.yaml` | No jump detection |
| Real-time Tracking | `high_speed.yaml` | Maximum FPS |
| Precision Measurement | `high_accuracy.yaml` | Strict filtering |
| General Use | `tag_localizer_params.yaml` | Balanced |

---

## Notes

- After modifying config files, you do **NOT** need to rebuild
- Simply restart the node to apply changes
- Config files are installed to `install/apriltag_detector/share/apriltag_detector/config/`
- You can monitor parameters in real-time:
  ```bash
  ros2 param list /tag_localizer_node
  ros2 param get /tag_localizer_node smoothing_alpha
  ```

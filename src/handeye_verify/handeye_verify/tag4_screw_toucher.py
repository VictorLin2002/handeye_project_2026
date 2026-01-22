#!/usr/bin/env python3
"""
Hand-Eye verification:
- Touch mode: move robot to computed screw/center targets
- Repeatability mode: collect N coherent samples of tag center in base frame

Key fix for repeatability:
- Each sample is defined as a coherent set of 4 corners
- Reset corner state for every sample
- Enforce per-sample timestamp coherence (avoid mixing frames)
"""

import time
import os
import importlib.util
from datetime import datetime
from typing import Tuple, Optional, List, Dict

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PointStamped
from rtde_controller_interfaces.action import MoveToPose

PLOTTING_AVAILABLE = False
PLOTTING_IMPORT_ERROR = "matplotlib not available"

if importlib.util.find_spec("matplotlib") is not None:
    from handeye_verify import plot_repeatability_errors

    create_visualization = plot_repeatability_errors.create_visualization
    create_simple_plot = plot_repeatability_errors.create_simple_plot
    compute_statistics = plot_repeatability_errors.compute_statistics
    PLOTTING_AVAILABLE = True


def normalize(v: np.ndarray, eps: float = 1e-12) -> Optional[np.ndarray]:
    """Normalize vector, return None if too small."""
    n = float(np.linalg.norm(v))
    if n < eps:
        return None
    return v / n


def rotmat_from_z_axis(tcp_z: np.ndarray, ref_x: Optional[np.ndarray] = None) -> Optional[np.ndarray]:
    """Build a right-handed rotation matrix whose Z axis equals tcp_z."""
    z = normalize(tcp_z)
    if z is None:
        return None

    if ref_x is None:
        ref_x = np.array([1.0, 0.0, 0.0], dtype=float)

    if abs(float(np.dot(z, ref_x))) > 0.95:
        ref_x = np.array([0.0, 1.0, 0.0], dtype=float)

    x = np.cross(ref_x, z)
    x = normalize(x)
    if x is None:
        ref_x = np.array([0.0, 1.0, 0.0], dtype=float)
        x = np.cross(ref_x, z)
        x = normalize(x)
        if x is None:
            return None

    y = np.cross(z, x)
    y = normalize(y)
    if y is None:
        return None

    x = np.cross(y, z)
    x = normalize(x)
    if x is None:
        return None

    return np.column_stack([x, y, z])


def matrix_to_axis_angle(R: np.ndarray) -> np.ndarray:
    """Convert rotation matrix to axis-angle rotation vector (rotvec)."""
    trace = float(np.trace(R))
    angle = float(np.arccos(np.clip((trace - 1.0) * 0.5, -1.0, 1.0)))

    if angle < 1e-8:
        return np.zeros(3, dtype=float)

    if abs(angle - np.pi) < 1e-6:
        axis = np.zeros(3, dtype=float)
        diag = np.diag(R)
        k = int(np.argmax(diag))
        axis[k] = np.sqrt(max((diag[k] + 1.0) * 0.5, 0.0))

        if axis[k] < 1e-8:
            return np.array([np.pi, 0.0, 0.0], dtype=float)

        if k == 0:
            axis[1] = R[0, 1] / (2.0 * axis[0])
            axis[2] = R[0, 2] / (2.0 * axis[0])
        elif k == 1:
            axis[0] = R[0, 1] / (2.0 * axis[1])
            axis[2] = R[1, 2] / (2.0 * axis[1])
        else:
            axis[0] = R[0, 2] / (2.0 * axis[2])
            axis[1] = R[1, 2] / (2.0 * axis[2])

        return axis * angle

    axis = np.array(
        [
            R[2, 1] - R[1, 2],
            R[0, 2] - R[2, 0],
            R[1, 0] - R[0, 1],
        ],
        dtype=float,
    ) / (2.0 * np.sin(angle))

    axis = normalize(axis)
    if axis is None:
        return np.zeros(3, dtype=float)

    return axis * angle


def axis_angle_to_matrix(rotvec: np.ndarray) -> np.ndarray:
    """Convert axis-angle rotation vector to rotation matrix."""
    angle = float(np.linalg.norm(rotvec))
    if angle < 1e-8:
        return np.eye(3, dtype=float)

    axis = rotvec / angle
    K = np.array(
        [
            [0.0, -axis[2], axis[1]],
            [axis[2], 0.0, -axis[0]],
            [-axis[1], axis[0], 0.0],
        ],
        dtype=float,
    )
    return np.eye(3, dtype=float) + np.sin(angle) * K + (1.0 - np.cos(angle)) * (K @ K)


class Tag4ScrewToucher(Node):
    def __init__(self):
        super().__init__("tag4_screw_toucher")

        # -----------------------------
        # Parameters: Repeatability test
        # -----------------------------
        self.declare_parameter("num_samples", 200)
        self.declare_parameter("sample_interval_sec", 0.05)          # spacing between samples
        self.declare_parameter("sample_timeout_sec", 1.0)           # time to wait for 4 corners per sample
        self.declare_parameter("stamp_coherence_ms", 5.0)           # max stamp spread among 4 corners
        self.declare_parameter("reference_point_mm", [0.0, -675.0, 83.5])
        self.declare_parameter("save_samples_csv", "")              # optional: save samples to CSV

        # Plotting parameters
        self.declare_parameter("auto_plot", True)                   # auto-generate plots after test
        self.declare_parameter("plot_mode", "full")                 # "full" | "simple" | "both"
        self.declare_parameter("output_dir", "/home/Victor/ros2_ws/results/handeye_calibration")

        # -----------------------------
        # Parameters: Topics
        # -----------------------------
        self.declare_parameter("corner0_topic", "/apriltag/tag4_corner0_3d")
        self.declare_parameter("corner1_topic", "/apriltag/tag4_corner1_3d")
        self.declare_parameter("corner2_topic", "/apriltag/tag4_corner2_3d")
        self.declare_parameter("corner3_topic", "/apriltag/tag4_corner3_3d")
        self.declare_parameter("center_topic", "/apriltag/tag4_center_3d")  # NEW: C++ center topic
        self.declare_parameter("action_name", "move_to_pose")

        # -----------------------------
        # Parameters: Rectangle screws + center
        # -----------------------------
        self.declare_parameter("rect_size_mm", 60.0)
        self.declare_parameter("screw_height_mm", 2.0)
        self.declare_parameter("screw_height_sign", +1.0)

        self.declare_parameter("include_center", True)
        self.declare_parameter("center_height_mm", 0.0)
        self.declare_parameter("center_height_sign", +1.0)

        self.declare_parameter("visit_order", ["screw0", "screw1", "screw2", "screw3", "center"])

        # -----------------------------
        # Parameters: Orientation
        # -----------------------------
        self.declare_parameter("orientation_mode", "plane_normal")  # plane_normal | vertical | camera_ray
        self.declare_parameter("tcp_z_sign", -1.0)

        # -----------------------------
        # Parameters: TCP Transformation
        # -----------------------------
        self.declare_parameter(
            "T_flange_tcp",
            [
                1.0, 0.0, 0.0, -0.000,
                0.0, 1.0, 0.0, -0.00,
                0.0, 0.0, 1.0, 0.2308,
                0.0, 0.0, 0.0, 1.0,
            ],
        )
        self.declare_parameter("tcp_pose_xyzrxryrz", [])

        # -----------------------------
        # Parameters: Motion
        # -----------------------------
        self.declare_parameter("approach_dist", 0.005)  # meters
        self.declare_parameter("visit_touch", True)

        self.declare_parameter("pause_each_point", True)
        self.declare_parameter("pause_mode", "time")    # manual | time | manual_or_time | none
        self.declare_parameter("dwell_sec", 2.0)

        self.declare_parameter("base_correction_xyz", [0.0, 0.0, 0.0])

        self.declare_parameter("z_min", 0.02)
        self.declare_parameter("z_max", 2.00)
        self.declare_parameter("speed", 0.10)
        self.declare_parameter("acc", 0.30)
        if not self.has_parameter("log_pose_detail"):
            self.declare_parameter("log_pose_detail", False)

        # -----------------------------
        # Load Transformations
        # -----------------------------
        self.declare_parameter("T_base_camera", [
            0.740591, 0.408351, -0.533642, 0.504284,
            0.671952, -0.452763, 0.586078, -0.989014,
            -0.002288, -0.792626, -0.609704, 0.655610,
            0.0, 0.0, 0.0, 1.0
        ])
        T_BC_list = self.get_parameter("T_base_camera").value
        self.T_BC = np.array(T_BC_list, dtype=float).reshape(4, 4)

        tcp_pose = self.get_parameter("tcp_pose_xyzrxryrz").value
        if tcp_pose and len(tcp_pose) == 6:
            self.T_FT = np.eye(4, dtype=float)
            self.T_FT[:3, 3] = np.array(tcp_pose[:3], dtype=float)
            self.T_FT[:3, :3] = axis_angle_to_matrix(np.array(tcp_pose[3:6], dtype=float))
            self.get_logger().info("Using TCP pose from tcp_pose_xyzrxryrz")
        else:
            T_FT_list = self.get_parameter("T_flange_tcp").value
            self.T_FT = np.array(T_FT_list, dtype=float).reshape(4, 4)
            self.get_logger().info("Using TCP transform from T_flange_tcp")

        self.camera_origin_B = self.T_BC[:3, 3].copy()

        # -----------------------------
        # State: corners and stamps
        # -----------------------------
        self.corners_C: List[Optional[np.ndarray]] = [None, None, None, None]
        self.corner_stamp_ns: List[Optional[int]] = [None, None, None, None]
        self.corners_received = [False, False, False, False]
        self.all_corners_received = False

        # NEW: center point from C++ (for repeatability mode)
        self.center_C: Optional[np.ndarray] = None
        self.center_stamp_ns: Optional[int] = None
        self.center_received = False

        # -----------------------------
        # ROS Interface
        # -----------------------------
        self.sub_corner0 = self.create_subscription(
            PointStamped, self.get_parameter("corner0_topic").value,
            lambda msg: self.corner_callback(0, msg), 10
        )
        self.sub_corner1 = self.create_subscription(
            PointStamped, self.get_parameter("corner1_topic").value,
            lambda msg: self.corner_callback(1, msg), 10
        )
        self.sub_corner2 = self.create_subscription(
            PointStamped, self.get_parameter("corner2_topic").value,
            lambda msg: self.corner_callback(2, msg), 10
        )
        self.sub_corner3 = self.create_subscription(
            PointStamped, self.get_parameter("corner3_topic").value,
            lambda msg: self.corner_callback(3, msg), 10
        )

        # NEW: subscribe to C++ center topic
        self.sub_center = self.create_subscription(
            PointStamped, self.get_parameter("center_topic").value,
            self.center_callback, 10
        )

        self.action_client = ActionClient(self, MoveToPose, self.get_parameter("action_name").value)
        self._log_config()

    # -----------------------------
    # Logging / state helpers
    # -----------------------------
    def _log_config(self):
        self.get_logger().info("=" * 70)
        self.get_logger().info("Tag4 Screw Toucher / Repeatability")
        self.get_logger().info("=" * 70)
        self.get_logger().info("T_BC (Base->Camera):")
        for row in self.T_BC:
            self.get_logger().info(f"  [{row[0]:+.6f}, {row[1]:+.6f}, {row[2]:+.6f}, {row[3]:+.6f}]")
        self.get_logger().info("T_FT (Flange->TCP):")
        for row in self.T_FT:
            self.get_logger().info(f"  [{row[0]:+.6f}, {row[1]:+.6f}, {row[2]:+.6f}, {row[3]:+.6f}]")
        self.get_logger().info("=" * 70)

    def _reset_corner_state(self):
        """Reset corner reception state for a new coherent sample."""
        self.corners_C = [None, None, None, None]
        self.corner_stamp_ns = [None, None, None, None]
        self.corners_received = [False, False, False, False]
        self.all_corners_received = False

    def corner_callback(self, idx: int, msg: PointStamped):
        """Store the latest corner point and its stamp."""
        self.corners_C[idx] = np.array([msg.point.x, msg.point.y, msg.point.z], dtype=float)
        stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        self.corner_stamp_ns[idx] = stamp_ns
        self.corners_received[idx] = True

        if all(self.corners_received):
            self.all_corners_received = True

    def center_callback(self, msg: PointStamped):
        """Store center point from C++ node (for repeatability mode)."""
        self.center_C = np.array([msg.point.x, msg.point.y, msg.point.z], dtype=float)
        stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        self.center_stamp_ns = stamp_ns
        self.center_received = True

    def _wait_for_coherent_corners(self, timeout_sec: float, max_stamp_spread_ns: int) -> bool:
        """
        Wait until 4 corners are received AND their stamps are coherent within a window.
        This prevents mixing corners from different frames/cycles.
        """
        t0 = time.time()
        while rclpy.ok() and (time.time() - t0 < timeout_sec):
            rclpy.spin_once(self, timeout_sec=0.01)

            if not self.all_corners_received:
                continue

            stamps = [s for s in self.corner_stamp_ns if s is not None]
            if len(stamps) != 4:
                continue

            spread = max(stamps) - min(stamps)
            if spread <= max_stamp_spread_ns:
                return True

            # Not coherent: keep waiting within the same timeout window
        return False

    def _wait_for_center(self, timeout_sec: float) -> bool:
        """Wait until center point is received from C++."""
        self.center_received = False
        t0 = time.time()
        while rclpy.ok() and (time.time() - t0 < timeout_sec):
            rclpy.spin_once(self, timeout_sec=0.01)
            if self.center_received:
                return True
        return False

    # -----------------------------
    # Geometry
    # -----------------------------
    def _corners_B(self) -> np.ndarray:
        corners_B = []
        for pC in self.corners_C:
            assert pC is not None
            pC_h = np.array([pC[0], pC[1], pC[2], 1.0], dtype=float)
            pB = (self.T_BC @ pC_h)[:3]
            corners_B.append(pB)
        return np.array(corners_B, dtype=float)

    def _tag_frame_B(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        corners_B = self._corners_B()
        center_B = np.mean(corners_B, axis=0)

        v1_C = self.corners_C[1] - self.corners_C[0]  # type: ignore
        v2_C = self.corners_C[2] - self.corners_C[0]  # type: ignore
        normal_C = normalize(np.cross(v1_C, v2_C))
        if normal_C is None:
            raise RuntimeError("Degenerate normal from corners (C frame).")

        normal_B = normalize(self.T_BC[:3, :3] @ normal_C)
        if normal_B is None:
            raise RuntimeError("Degenerate normal after transform (B frame).")

        x_raw = corners_B[1] - corners_B[0]
        x_raw = x_raw - normal_B * float(np.dot(x_raw, normal_B))
        x_axis_B = normalize(x_raw)
        if x_axis_B is None:
            raise RuntimeError("Degenerate x axis from corners.")

        y_axis_B = normalize(np.cross(normal_B, x_axis_B))
        if y_axis_B is None:
            raise RuntimeError("Degenerate y axis from frame construction.")

        return center_B, x_axis_B, y_axis_B, normal_B

    # -----------------------------
    # Touch-mode helpers
    # -----------------------------
    def _compute_tcp_z_B(self, target_B: np.ndarray, normal_B: np.ndarray) -> Optional[np.ndarray]:
        mode = str(self.get_parameter("orientation_mode").value).strip().lower()
        sign = float(self.get_parameter("tcp_z_sign").value)
        sign = +1.0 if sign >= 0.0 else -1.0

        if mode == "plane_normal":
            return normal_B * sign

        if mode == "camera_ray":
            v = target_B - self.camera_origin_B
            z = normalize(v)
            if z is None:
                return None
            return z * sign

        return np.array([0.0, 0.0, -1.0], dtype=float) * sign

    def _compute_flange_pose6_B(self, tip_target_B: np.ndarray, tcp_z_desired_B: np.ndarray) -> Optional[np.ndarray]:
        R_BT = rotmat_from_z_axis(tcp_z_desired_B)
        if R_BT is None:
            return None

        R_FT = self.T_FT[:3, :3]
        R_TF = R_FT.T
        R_BF = R_BT @ R_TF

        t_FT = self.T_FT[:3, 3]
        flange_pos_B = tip_target_B - R_BF @ t_FT

        rotvec_BF = matrix_to_axis_angle(R_BF)
        return np.concatenate([flange_pos_B, rotvec_BF]).astype(float)

    def _safety_check(self, flange_pos_B: np.ndarray) -> bool:
        z_min = float(self.get_parameter("z_min").value)
        z_max = float(self.get_parameter("z_max").value)
        z = float(flange_pos_B[2])

        if not np.isfinite(z):
            self.get_logger().error("Invalid flange Z (NaN/Inf).")
            return False

        if z < z_min or z > z_max:
            self.get_logger().error(f"Workspace check failed: z={z:.3f} not in [{z_min:.3f}, {z_max:.3f}]")
            return False

        return True

    def _send_pose6(self, pose6: np.ndarray, label: str) -> bool:
        speed = float(self.get_parameter("speed").value)
        acc = float(self.get_parameter("acc").value)

        goal = MoveToPose.Goal()
        goal.x, goal.y, goal.z = map(float, pose6[:3])
        goal.rx, goal.ry, goal.rz = map(float, pose6[3:])
        goal.speed = float(speed)
        goal.acc = float(acc)

        if bool(self.get_parameter("log_pose_detail").value):
            self.get_logger().info(
                f"[{label}] pose6: x={pose6[0]:+.6f}, y={pose6[1]:+.6f}, z={pose6[2]:+.6f}, "
                f"rx={pose6[3]:+.4f}, ry={pose6[4]:+.4f}, rz={pose6[5]:+.4f}"
            )
        else:
            self.get_logger().info(f"[{label}] sending motion command")

        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action server not available.")
            return False

        send_future = self.action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or (not goal_handle.accepted):
            self.get_logger().error("Goal rejected.")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        res = result_future.result()
        if res is None:
            self.get_logger().error("No result returned.")
            return False

        result = res.result
        ok = bool(getattr(result, "success", False))
        msg = str(getattr(result, "message", ""))
        if ok:
            self.get_logger().info(f"[{label}] SUCCESS: {msg}")
        else:
            self.get_logger().error(f"[{label}] FAILED: {msg}")
        return ok

    def _pause_here(self, label: str):
        """Pause at each point using configured mode."""
        if not bool(self.get_parameter("pause_each_point").value):
            return

        mode = str(self.get_parameter("pause_mode").value).strip().lower()
        dwell = float(self.get_parameter("dwell_sec").value)

        if mode == "none":
            return

        did_time = False
        if mode in ("time", "manual_or_time") and dwell > 0.0:
            self.get_logger().info(f"[{label}] dwell {dwell:.3f} sec")
            time.sleep(dwell)
            did_time = True

        if mode in ("manual", "manual_or_time") and (not did_time or mode == "manual"):
            try:
                input(f"[{label}] Press Enter to continue...")
            except EOFError:
                pass

    def _compute_targets_B(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, Dict[str, np.ndarray]]:
        """Compute 4 screw targets + optional center in base frame."""
        center_B, x_axis_B, y_axis_B, normal_B = self._tag_frame_B()

        corr = np.array(self.get_parameter("base_correction_xyz").value, dtype=float)
        center_B_corr = center_B + corr

        rect_size_m = float(self.get_parameter("rect_size_mm").value) * 0.001
        half = 0.5 * rect_size_m

        screw_h_m = float(self.get_parameter("screw_height_mm").value) * 0.001
        screw_sign = +1.0 if float(self.get_parameter("screw_height_sign").value) >= 0.0 else -1.0
        screw_height_vec = normal_B * (screw_sign * screw_h_m)

        xy = [(+half, +half), (+half, -half), (-half, -half), (-half, +half)]

        targets: Dict[str, np.ndarray] = {}
        for i, (dx, dy) in enumerate(xy):
            targets[f"screw{i}"] = center_B_corr + x_axis_B * dx + y_axis_B * dy + screw_height_vec

        if bool(self.get_parameter("include_center").value):
            center_h_m = float(self.get_parameter("center_height_mm").value) * 0.001
            center_sign = +1.0 if float(self.get_parameter("center_height_sign").value) >= 0.0 else -1.0
            center_height_vec = normal_B * (center_sign * center_h_m)
            targets["center"] = center_B_corr + center_height_vec

        return center_B_corr, x_axis_B, y_axis_B, normal_B, targets

    # -----------------------------
    # Modes
    # -----------------------------
    def run_once_touch(self) -> bool:
        """One cycle: wait coherent corners, compute targets, move sequentially."""
        self._reset_corner_state()

        timeout = float(self.get_parameter("sample_timeout_sec").value)
        max_spread_ns = int(float(self.get_parameter("stamp_coherence_ms").value) * 1e6)

        if not self._wait_for_coherent_corners(timeout, max_spread_ns):
            self.get_logger().error("Failed to get coherent 4 corners for touch mode.")
            return False

        try:
            center_B_corr, x_axis_B, y_axis_B, normal_B, targets = self._compute_targets_B()
        except Exception as e:
            self.get_logger().error(f"Failed to compute targets: {e}")
            return False

        order = list(self.get_parameter("visit_order").value)
        if len(order) == 0:
            self.get_logger().error("visit_order is empty.")
            return False
        for name in order:
            if name not in targets:
                self.get_logger().error(f"visit_order item '{name}' not in targets: {list(targets.keys())}")
                return False

        self.get_logger().info("=" * 70)
        self.get_logger().info("Targets (Base frame)")
        self.get_logger().info("=" * 70)
        self.get_logger().info(
            f"center_B_corr(mm)=({center_B_corr[0]*1000:+.2f},{center_B_corr[1]*1000:+.2f},{center_B_corr[2]*1000:+.2f})"
        )
        for k in order:
            p = targets[k]
            self.get_logger().info(f"{k}_B(mm)=({p[0]*1000:+.2f},{p[1]*1000:+.2f},{p[2]*1000:+.2f})")
        self.get_logger().info("=" * 70)

        approach_dist = float(self.get_parameter("approach_dist").value)
        do_touch = bool(self.get_parameter("visit_touch").value)

        for idx, name in enumerate(order):
            p_touch = targets[name]
            tcp_z = self._compute_tcp_z_B(p_touch, normal_B)
            if tcp_z is None:
                self.get_logger().error("Failed to compute tcp_z.")
                return False

            p_approach = p_touch - tcp_z * approach_dist

            pose6_app = self._compute_flange_pose6_B(p_approach, tcp_z)
            if pose6_app is None:
                self.get_logger().error("Failed to compute approach pose6.")
                return False
            if not self._safety_check(pose6_app[:3]):
                return False
            if not self._send_pose6(pose6_app, f"P{idx}:{name}:APPROACH"):
                return False
            self._pause_here(f"P{idx}:{name}:APPROACH")

            if do_touch:
                pose6_touch = self._compute_flange_pose6_B(p_touch, tcp_z)
                if pose6_touch is None:
                    self.get_logger().error("Failed to compute touch pose6.")
                    return False
                if not self._safety_check(pose6_touch[:3]):
                    return False
                if not self._send_pose6(pose6_touch, f"P{idx}:{name}:TOUCH"):
                    return False
                self._pause_here(f"P{idx}:{name}:TOUCH")

                if not self._send_pose6(pose6_app, f"P{idx}:{name}:RETREAT"):
                    return False
                self._pause_here(f"P{idx}:{name}:RETREAT")

        return True

    def _generate_plots(self, samples_mm: np.ndarray, ref_mm: np.ndarray, csv_path: str):
        """Generate analysis plots after repeatability test."""
        plot_mode = str(self.get_parameter("plot_mode").value).strip().lower()
        output_dir = str(self.get_parameter("output_dir").value).strip()

        # Create plots subdirectory
        plots_dir = os.path.join(output_dir, "plots")
        os.makedirs(plots_dir, exist_ok=True)

        # Generate timestamp for filename
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        self.get_logger().info("=" * 70)
        self.get_logger().info("Generating analysis plots...")
        self.get_logger().info("=" * 70)

        if plot_mode in ("full", "both"):
            full_path = os.path.join(plots_dir, f"calibration_full_{timestamp}.png")
            self.get_logger().info(f"Creating full analysis plot (6 panels)...")
            create_visualization(samples_mm, ref_mm, full_path)
            self.get_logger().info(f"✓ Full analysis plot saved: {full_path}")

        if plot_mode in ("simple", "both"):
            simple_path = os.path.join(plots_dir, f"calibration_simple_{timestamp}.png")
            self.get_logger().info(f"Creating simple plot (2 panels)...")
            create_simple_plot(samples_mm, ref_mm, simple_path)
            self.get_logger().info(f"✓ Simple plot saved: {simple_path}")

        self.get_logger().info("=" * 70)

    def run_repeatability(self) -> bool:
        """Collect N samples of tag center from C++ node in base frame."""
        num = int(self.get_parameter("num_samples").value)
        interval = float(self.get_parameter("sample_interval_sec").value)
        timeout = float(self.get_parameter("sample_timeout_sec").value)

        ref_mm = np.array(self.get_parameter("reference_point_mm").value, dtype=float)

        centers_B: List[np.ndarray] = []

        for i in range(num):
            # Wait for new center point from C++
            ok = self._wait_for_center(timeout)
            if not ok:
                self.get_logger().warn(f"[Repeatability] sample {i+1}/{num}: timeout waiting for center")
                time.sleep(max(0.0, interval))
                continue

            # Transform center from camera to base frame
            assert self.center_C is not None
            pC_h = np.array([self.center_C[0], self.center_C[1], self.center_C[2], 1.0], dtype=float)
            center_B = (self.T_BC @ pC_h)[:3]
            centers_B.append(center_B.copy())

            time.sleep(max(0.0, interval))

        if len(centers_B) < 3:
            self.get_logger().error(f"[Repeatability] insufficient valid samples: {len(centers_B)}/{num}")
            return False

        centers_mm = np.array(centers_B) * 1000.0
        mean_mm = centers_mm.mean(axis=0)
        std_mm = centers_mm.std(axis=0, ddof=1)
        bias_mm = mean_mm - ref_mm
        norm_bias_mm = float(np.linalg.norm(bias_mm))

        self.get_logger().info("=" * 70)
        self.get_logger().info("REPEATABILITY RESULT (Tag center from C++)")
        self.get_logger().info("=" * 70)
        self.get_logger().info(f"samples_valid={len(centers_B)}/{num}")
        self.get_logger().info(
            f"mean(mm)=({mean_mm[0]:+.3f},{mean_mm[1]:+.3f},{mean_mm[2]:+.3f})"
        )
        self.get_logger().info(
            f"std(mm)=({std_mm[0]:.3f},{std_mm[1]:.3f},{std_mm[2]:.3f})"
        )
        self.get_logger().info(
            f"bias(mm)=({bias_mm[0]:+.3f},{bias_mm[1]:+.3f},{bias_mm[2]:+.3f}), norm={norm_bias_mm:.3f}"
        )
        self.get_logger().info("=" * 70)

        # Save samples to CSV
        csv_path = str(self.get_parameter("save_samples_csv").value).strip()
        if not csv_path:
            # Auto-generate path if not specified
            output_dir = str(self.get_parameter("output_dir").value).strip()
            data_dir = os.path.join(output_dir, "data")
            os.makedirs(data_dir, exist_ok=True)
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            csv_path = os.path.join(data_dir, f"calibration_samples_{timestamp}.csv")

        try:
            import csv
            with open(csv_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['sample_idx', 'x_mm', 'y_mm', 'z_mm'])
                for i, center in enumerate(centers_mm):
                    writer.writerow([i, center[0], center[1], center[2]])
            self.get_logger().info(f"Samples saved to: {csv_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to save samples: {e}")
            csv_path = ""  # Clear path if save failed

        # Generate plots if enabled
        if bool(self.get_parameter("auto_plot").value) and csv_path:
            if not PLOTTING_AVAILABLE:
                self.get_logger().warn(
                    f"Plotting requested but matplotlib not available: {PLOTTING_IMPORT_ERROR}. "
                    "Install matplotlib to enable plotting: pip3 install matplotlib"
                )
            else:
                try:
                    self._generate_plots(centers_mm, ref_mm, csv_path)
                except Exception as e:
                    self.get_logger().error(f"Failed to generate plots: {e}")

        return True

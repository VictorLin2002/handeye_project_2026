#!/usr/bin/env python3
import csv
import time
import math
import threading
from collections import deque
from typing import Deque, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Int64MultiArray

from rclpy.action import ActionServer
from handeye_logger_interfaces.action import CaptureHandEye

from rtde_receive import RTDEReceiveInterface


def quat_dot(q1: Tuple[float, float, float, float],
             q2: Tuple[float, float, float, float]) -> float:
    return q1[0]*q2[0] + q1[1]*q2[1] + q1[2]*q2[2] + q1[3]*q2[3]


def quat_angle_distance_rad(q1: Tuple[float, float, float, float],
                            q2: Tuple[float, float, float, float]) -> float:
    """
    Angular distance (rad) between two quaternions.
    Uses the double-cover property by taking abs(dot).
    """
    d = abs(quat_dot(q1, q2))
    d = max(-1.0, min(1.0, d))
    return 2.0 * math.acos(d)


class HandEyeLogger(Node):
    """
    Hand-eye data logger.

    Key behaviors:
      - Service-driven capture: /handeye_logger/capture_once
      - Time sync: use vision msg stamp as the query time to synchronize TCP (nearest/interpolated)
      - Robustness: publish status even on failures for external scripts to diagnose
    """

    def __init__(self):
        super().__init__("handeye_logger")

        # Connection / IO
        self.declare_parameter("robot_ip", "192.168.0.200")
        self.declare_parameter("rtde_frequency", 500)
        self.declare_parameter("object_pose_topic", "/apriltag/object_pose")
        self.declare_parameter("csv_path", "handeye_samples.csv")

        # Stationary detection (robot)
        self.declare_parameter("stationary_window", 100)
        self.declare_parameter("pos_eps_m", 0.0001)
        self.declare_parameter("rot_eps_rad", 0.003)
        self.declare_parameter("rearm_pos_m", 0.001)
        self.declare_parameter("rearm_rot_rad", 0.01)
        self.declare_parameter("rtde_poll_hz", 500.0)

        # Vision stability (optional)
        self.declare_parameter("enable_vision_stability_check", False)
        self.declare_parameter("vision_stable_window", 2)
        self.declare_parameter("vision_pos_eps_m", 0.005)
        self.declare_parameter("vision_ang_eps_deg", 1.0)

        # Reprojection error filtering (RGB quality gate, in pixels)
        # Note: covariance[0] now contains reprojection_error_px from PnP, not RMSE
        self.declare_parameter("max_reprojection_error_px", 2.0)
        self.declare_parameter("reprojection_reject_limit", 30)

        # Warmup
        self.declare_parameter("warmup_s", 0.5)

        # Status publishing
        self.declare_parameter("status_topic", "/handeye_logger/status")

        # Capture action (recommended over auto_log)
        self.declare_parameter("auto_log", False)
        self.declare_parameter("capture_action", "/handeye_logger/capture_once")
        self.declare_parameter("capture_timeout_s", 1.5)

        # Capture gating: how many NEW vision messages required after capture request
        # Set to 1 for maximum robustness; increase to 2~3 only if your vision stream is very stable.
        self.declare_parameter("capture_min_new_vision_msgs", 1)

        # Time synchronization
        self.declare_parameter("max_sync_dt_s", 0.04)
        self.declare_parameter("max_vision_age_s", 0.30)
        self.declare_parameter("enable_tcp_interpolation", True)

        # Load params
        self.robot_ip = str(self.get_parameter("robot_ip").value)
        self.rtde_frequency = int(self.get_parameter("rtde_frequency").value)
        self.object_pose_topic = str(self.get_parameter("object_pose_topic").value)
        self.csv_path = str(self.get_parameter("csv_path").value)

        self.stationary_window = int(self.get_parameter("stationary_window").value)
        self.pos_eps_m = float(self.get_parameter("pos_eps_m").value)
        self.rot_eps_rad = float(self.get_parameter("rot_eps_rad").value)
        self.rearm_pos_m = float(self.get_parameter("rearm_pos_m").value)
        self.rearm_rot_rad = float(self.get_parameter("rearm_rot_rad").value)
        self.rtde_poll_hz = float(self.get_parameter("rtde_poll_hz").value)

        self.enable_vision_stability_check = bool(self.get_parameter("enable_vision_stability_check").value)
        self.vision_stable_window = int(self.get_parameter("vision_stable_window").value)
        self.vision_pos_eps_m = float(self.get_parameter("vision_pos_eps_m").value)
        self.vision_ang_eps_rad = math.radians(float(self.get_parameter("vision_ang_eps_deg").value))

        self.max_reprojection_error_px = float(self.get_parameter("max_reprojection_error_px").value)
        self.reprojection_reject_limit = int(self.get_parameter("reprojection_reject_limit").value)
        self.warmup_s = float(self.get_parameter("warmup_s").value)

        self.status_topic = str(self.get_parameter("status_topic").value)

        self.auto_log = bool(self.get_parameter("auto_log").value)
        self.capture_action_name = str(self.get_parameter("capture_action").value)
        self.capture_timeout_s = float(self.get_parameter("capture_timeout_s").value)
        self.capture_min_new_vision_msgs = int(self.get_parameter("capture_min_new_vision_msgs").value)

        self.max_sync_dt_s = float(self.get_parameter("max_sync_dt_s").value)
        self.max_vision_age_s = float(self.get_parameter("max_vision_age_s").value)
        self.enable_tcp_interpolation = bool(self.get_parameter("enable_tcp_interpolation").value)

        self._start_time = time.time()
        self._rmse_reject_count = 0

        # RTDE
        self.rtde_r = RTDEReceiveInterface(self.robot_ip, self.rtde_frequency)
        self.get_logger().info(f"RTDE connected: {self.robot_ip} @ {self.rtde_frequency} Hz")
        self.get_logger().info(f"Vision topic: {self.object_pose_topic}")
        self.get_logger().info(f"CSV path: {self.csv_path}")
        self.get_logger().info(f"Reprojection error filter: max {self.max_reprojection_error_px:.2f} px")

        # CSV output
        self.f = open(self.csv_path, "w", newline="")
        self.w = csv.writer(self.f)
        self.w.writerow([
            "host_stamp_sec",
            "obj_msg_stamp_sec",
            "B_tcp_x", "B_tcp_y", "B_tcp_z", "B_tcp_rx", "B_tcp_ry", "B_tcp_rz",
            "C_p_O_x", "C_p_O_y", "C_p_O_z", "C_q_O_x", "C_q_O_y", "C_q_O_z", "C_q_O_w",
        ])
        self.f.flush()

        # Status publisher: make it latched-like so late subscribers can see last state
        status_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.status_pub = self.create_publisher(Int64MultiArray, self.status_topic, status_qos)

        self.get_logger().info(f"Status topic: {self.status_topic} (TRANSIENT_LOCAL)")
        self.get_logger().info(f"Auto log: {'ON' if self.auto_log else 'OFF'}")
        self.get_logger().info(f"Capture action: {self.capture_action_name}")
        self.get_logger().info(
            f"Capture timeout: {self.capture_timeout_s:.2f}s | min_new_vision_msgs: {self.capture_min_new_vision_msgs}"
        )
        self.get_logger().info(
            f"Time sync: dt<{self.max_sync_dt_s*1000:.0f}ms | vision_age<{self.max_vision_age_s*1000:.0f}ms"
        )
        self.get_logger().info(f"TCP interpolation: {'ON' if self.enable_tcp_interpolation else 'OFF'}")

        # Buffers
        self.tcp_hist: Deque[Tuple[float, float, float, float, float, float]] = deque(maxlen=self.stationary_window)
        tcp_time_hist_len = int(max(200, self.rtde_poll_hz * 3.0))
        self.tcp_time_hist: Deque[Tuple[float, Tuple[float, float, float, float, float, float]]] = deque(maxlen=tcp_time_hist_len)

        self.latest_tcp: Optional[Tuple[float, float, float, float, float, float]] = None
        self.latest_tcp_time: Optional[float] = None

        self.vision_hist: Deque[PoseWithCovarianceStamped] = deque(maxlen=max(1, self.vision_stable_window))
        self.latest_obj_pose: Optional[PoseWithCovarianceStamped] = None

        # Track message arrival (receive time) separately from ROS stamp
        self.latest_obj_recv_time: Optional[float] = None

        # State machine (auto_log)
        self.armed = True
        self.logged_count = 0
        self.lock_ref_tcp: Optional[Tuple[float, float, float, float, float, float]] = None

        self._last_logged_obj_stamp_us: int = -1
        self._last_logged_tcp: Optional[Tuple[float, float, float, float, float, float]] = None
        self._min_tcp_distance_m: float = 0.002
        self._min_tcp_rotation_rad: float = 0.005  # ~0.29 degrees

        # Capture service state
        self._capture_lock = threading.Lock()
        self._capture_armed = False
        self._capture_req_recv_time: Optional[float] = None
        self._capture_req_stamp_sec: Optional[float] = None  # ROS stamp threshold (vision stamp must be newer)
        self._capture_event = threading.Event()

        self._capture_pose: Optional[PoseWithCovarianceStamped] = None
        self._capture_pose_recv_time: Optional[float] = None

        self._capture_new_vision_count = 0

        # Subscriptions / timers / action server
        self.sub = self.create_subscription(PoseWithCovarianceStamped, self.object_pose_topic, self.vision_cb, 10)
        self._action_server = ActionServer(
            self,
            CaptureHandEye,
            self.capture_action_name,
            self.execute_capture_callback
        )

        period = 1.0 / max(1.0, self.rtde_poll_hz)
        self.timer = self.create_timer(period, self.poll_rtde)

        # Publish initial status
        self._publish_status(obj_stamp_sec=0.0)

    @staticmethod
    def _stamp_to_sec(msg: PoseWithCovarianceStamped) -> float:
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def _publish_status(self, obj_stamp_sec: float = 0.0):
        """
        Publish [logged_count, obj_stamp_us] for external scripts.

        Note:
          - TRANSIENT_LOCAL durability allows late subscribers to see the last published status.
        """
        msg = Int64MultiArray()
        msg.data = [
            int(self.logged_count),
            int(round(obj_stamp_sec * 1e6)) if obj_stamp_sec > 0.0 else 0,
        ]
        self.status_pub.publish(msg)

    def vision_cb(self, msg: PoseWithCovarianceStamped):
        """
        Store latest vision pose.
        Uses wall-clock receive time for timeout/age logic, and ROS stamp for synchronization.
        """
        recv_time = time.time()
        self.latest_obj_pose = msg
        self.latest_obj_recv_time = recv_time
        self.vision_hist.append(msg)

        with self._capture_lock:
            if not self._capture_armed:
                return

            vision_stamp_sec = self._stamp_to_sec(msg)

            # Require the vision stamp to be newer than the capture request stamp (prevents using stale latched frames)
            if self._capture_req_stamp_sec is not None and vision_stamp_sec <= self._capture_req_stamp_sec:
                return

            self._capture_new_vision_count += 1
            if self._capture_new_vision_count < self.capture_min_new_vision_msgs:
                return

            # Optional stability check
            if self.enable_vision_stability_check and not self._is_vision_stable():
                return

            self._capture_pose = msg
            self._capture_pose_recv_time = recv_time
            self._capture_event.set()

    def poll_rtde(self):
        """Poll RTDE pose, update stationary/motion state, and optionally auto log."""
        if (time.time() - self._start_time) < self.warmup_s:
            return

        t_before = time.time()
        try:
            tcp = self.rtde_r.getActualTCPPose()
            tcp6 = (float(tcp[0]), float(tcp[1]), float(tcp[2]),
                    float(tcp[3]), float(tcp[4]), float(tcp[5]))
        except Exception as e:
            self.get_logger().error(f"RTDE read failed: {e}")
            return
        t_after = time.time()
        t_mid = (t_before + t_after) / 2.0

        self.latest_tcp = tcp6
        self.latest_tcp_time = t_mid

        self.tcp_hist.append(tcp6)
        self.tcp_time_hist.append((t_mid, tcp6))

        # Auto log mode (optional)
        if not self.auto_log:
            return

        if not self.armed:
            if self._has_moved_from_locked_reference():
                self.armed = True
                self._rmse_reject_count = 0
                self.get_logger().info("Re-armed: robot moved.")
                self._publish_status(obj_stamp_sec=0.0)
            return

        if self._is_robot_stationary() and len(self.vision_hist) >= max(1, self.vision_stable_window):
            if self.latest_obj_pose is None:
                return
            self._log_once(self.latest_obj_pose, tcp6)

    def _get_nearest_tcp(self, t_query: float) -> Optional[Tuple[float, Tuple[float, float, float, float, float, float]]]:
        """
        Return (t_tcp, tcp6) synchronized to t_query (wall-clock domain).

        Uses linear interpolation when enabled.
        """
        if len(self.tcp_time_hist) == 0:
            return None
        if len(self.tcp_time_hist) == 1:
            return self.tcp_time_hist[0]

        if not self.enable_tcp_interpolation:
            return min(self.tcp_time_hist, key=lambda x: abs(x[0] - t_query))

        tcp_list = sorted(self.tcp_time_hist, key=lambda x: x[0])

        insert_idx = 0
        for i, (t, _) in enumerate(tcp_list):
            if t >= t_query:
                insert_idx = i
                break
        else:
            insert_idx = len(tcp_list)

        if insert_idx == 0:
            return tcp_list[0]
        if insert_idx == len(tcp_list):
            return tcp_list[-1]

        t0, tcp0 = tcp_list[insert_idx - 1]
        t1, tcp1 = tcp_list[insert_idx]

        if (t1 - t0) > 0.1:
            return (t0, tcp0) if abs(t_query - t0) < abs(t_query - t1) else (t1, tcp1)

        alpha = (t_query - t0) / (t1 - t0)
        alpha = max(0.0, min(1.0, alpha))

        x = tcp0[0] + alpha * (tcp1[0] - tcp0[0])
        y = tcp0[1] + alpha * (tcp1[1] - tcp0[1])
        z = tcp0[2] + alpha * (tcp1[2] - tcp0[2])
        rx = tcp0[3] + alpha * (tcp1[3] - tcp0[3])
        ry = tcp0[4] + alpha * (tcp1[4] - tcp0[4])
        rz = tcp0[5] + alpha * (tcp1[5] - tcp0[5])

        return (t_query, (x, y, z, rx, ry, rz))

    def execute_capture_callback(self, goal_handle):
        """
        Action callback to capture one synchronized sample on demand.

        Design:
          - Uses wall-clock receive time for waiting/age checks.
          - Uses vision message stamp for pairing with TCP time domain via nearest/interpolation.
          - Non-blocking: runs in action server thread, can provide feedback.
        """
        feedback_msg = CaptureHandEye.Feedback()
        result = CaptureHandEye.Result()

        # Latch capture request
        req_recv_time = time.time()
        req_stamp_sec = self.get_clock().now().nanoseconds * 1e-9  # ROS time now for "newer stamp" threshold

        with self._capture_lock:
            self._capture_event.clear()
            self._capture_pose = None
            self._capture_pose_recv_time = None

            self._capture_req_recv_time = req_recv_time
            self._capture_req_stamp_sec = req_stamp_sec

            self._capture_new_vision_count = 0
            self._capture_armed = True

        # Quick stationary check (action-driven capture assumes robot already settled)
        feedback_msg.status = "Checking robot stationary..."
        goal_handle.publish_feedback(feedback_msg)

        if not self._is_robot_stationary():
            with self._capture_lock:
                self._capture_armed = False
            result.success = False
            result.message = "Robot not stationary."
            result.obj_stamp_sec = 0.0
            self._publish_status(obj_stamp_sec=0.0)
            goal_handle.abort()
            return result

        # Wait for new vision messages
        feedback_msg.status = "Waiting for vision data..."
        goal_handle.publish_feedback(feedback_msg)

        if not self._capture_event.wait(timeout=self.capture_timeout_s):
            with self._capture_lock:
                self._capture_armed = False
            result.success = False
            result.message = f"Vision timeout ({self.capture_timeout_s:.2f}s)."
            result.obj_stamp_sec = 0.0
            self._publish_status(obj_stamp_sec=0.0)
            goal_handle.abort()
            return result

        with self._capture_lock:
            self._capture_armed = False
            pose_msg = self._capture_pose
            pose_recv_time = self._capture_pose_recv_time

        if pose_msg is None or pose_recv_time is None:
            result.success = False
            result.message = "Internal error: missing pose."
            result.obj_stamp_sec = 0.0
            self._publish_status(obj_stamp_sec=0.0)
            goal_handle.abort()
            return result

        # Vision age check uses receive time (network/queue delay)
        feedback_msg.status = "Validating vision data..."
        goal_handle.publish_feedback(feedback_msg)

        age_s = time.time() - pose_recv_time
        if age_s > self.max_vision_age_s:
            result.success = False
            result.message = f"Vision too old: {age_s:.3f}s."
            result.obj_stamp_sec = 0.0
            self._publish_status(obj_stamp_sec=0.0)
            goal_handle.abort()
            return result

        # Pair TCP with the vision stamp time (we assume both are in the same wall-clock domain when using system time stamps)
        feedback_msg.status = "Synchronizing TCP and vision data..."
        goal_handle.publish_feedback(feedback_msg)

        obj_stamp_sec = self._stamp_to_sec(pose_msg)
        nearest = self._get_nearest_tcp(obj_stamp_sec)
        if nearest is None:
            result.success = False
            result.message = "No TCP samples available."
            result.obj_stamp_sec = 0.0
            self._publish_status(obj_stamp_sec=0.0)
            goal_handle.abort()
            return result

        t_tcp, tcp_sync = nearest
        dt = abs(t_tcp - obj_stamp_sec)
        if dt > self.max_sync_dt_s:
            result.success = False
            result.message = f"Sync dt too large: {dt*1000:.1f}ms."
            result.obj_stamp_sec = 0.0
            self._publish_status(obj_stamp_sec=0.0)
            goal_handle.abort()
            return result

        feedback_msg.status = "Logging sample..."
        goal_handle.publish_feedback(feedback_msg)

        ok = self._log_once(pose_msg, tcp_sync, host_stamp_override=t_tcp)
        if not ok:
            result.success = False
            result.message = "Sample rejected (RMSE or dedupe)."
            result.obj_stamp_sec = 0.0
            self._publish_status(obj_stamp_sec=0.0)
            goal_handle.abort()
            return result

        result.success = True
        result.message = "Logged."
        result.obj_stamp_sec = obj_stamp_sec
        # _log_once publishes status with obj stamp
        goal_handle.succeed()
        return result

    def _is_robot_stationary(self) -> bool:
        """Check if TCP pose is stable within threshold."""
        if len(self.tcp_hist) < self.stationary_window:
            return False

        xs = [p[0] for p in self.tcp_hist]
        ys = [p[1] for p in self.tcp_hist]
        zs = [p[2] for p in self.tcp_hist]
        rxs = [p[3] for p in self.tcp_hist]
        rys = [p[4] for p in self.tcp_hist]
        rzs = [p[5] for p in self.tcp_hist]

        dx = max(xs) - min(xs)
        dy = max(ys) - min(ys)
        dz = max(zs) - min(zs)
        dpos = math.sqrt(dx*dx + dy*dy + dz*dz)

        drx = max(rxs) - min(rxs)
        dry = max(rys) - min(rys)
        drz = max(rzs) - min(rzs)
        drot = math.sqrt(drx*drx + dry*dry + drz*drz)

        return (dpos <= self.pos_eps_m) and (drot <= self.rot_eps_rad)

    def _has_moved_from_locked_reference(self) -> bool:
        """Check if robot moved enough from locked reference to re-arm."""
        if self.latest_tcp is None or self.lock_ref_tcp is None:
            return False

        ref = self.lock_ref_tcp
        cur = self.latest_tcp

        dpos = math.sqrt((cur[0] - ref[0])**2 +
                         (cur[1] - ref[1])**2 +
                         (cur[2] - ref[2])**2)
        drot = math.sqrt((cur[3] - ref[3])**2 +
                         (cur[4] - ref[4])**2 +
                         (cur[5] - ref[5])**2)

        return (dpos >= self.rearm_pos_m) or (drot >= self.rearm_rot_rad)

    def _is_vision_stable(self) -> bool:
        """Optional vision stability check over the current vision_hist window."""
        if len(self.vision_hist) < max(1, self.vision_stable_window):
            return False

        ref_pose = self.vision_hist[0].pose.pose
        ref_p = (ref_pose.position.x, ref_pose.position.y, ref_pose.position.z)
        ref_q = (ref_pose.orientation.x, ref_pose.orientation.y, ref_pose.orientation.z, ref_pose.orientation.w)

        for m in list(self.vision_hist)[1:]:
            p = m.pose.pose.position
            q = m.pose.pose.orientation

            dp = math.sqrt((p.x - ref_p[0])**2 + (p.y - ref_p[1])**2 + (p.z - ref_p[2])**2)
            dq = quat_angle_distance_rad((q.x, q.y, q.z, q.w), ref_q)

            if dp > self.vision_pos_eps_m or dq > self.vision_ang_eps_rad:
                return False

        return True

    def _log_once(
        self,
        msg: PoseWithCovarianceStamped,
        tcp6: Tuple[float, float, float, float, float, float],
        host_stamp_override: Optional[float] = None,
    ) -> bool:
        """Write one sample to CSV."""
        # covariance[0] now contains reprojection_error_px (RGB PnP quality metric)
        reprojection_error_px = float(msg.pose.covariance[0])
        obj_stamp = self._stamp_to_sec(msg)
        obj_stamp_us = int(round(obj_stamp * 1e6))

        # Deduplicate by vision timestamp
        if obj_stamp_us == self._last_logged_obj_stamp_us:
            return False

        # Deduplicate by TCP position AND rotation (prevent logging same pose twice)
        if self._last_logged_tcp is not None:
            dx = tcp6[0] - self._last_logged_tcp[0]
            dy = tcp6[1] - self._last_logged_tcp[1]
            dz = tcp6[2] - self._last_logged_tcp[2]
            tcp_dist = math.sqrt(dx*dx + dy*dy + dz*dz)

            drx = tcp6[3] - self._last_logged_tcp[3]
            dry = tcp6[4] - self._last_logged_tcp[4]
            drz = tcp6[5] - self._last_logged_tcp[5]
            tcp_rot = math.sqrt(drx*drx + dry*dry + drz*drz)

            # Only reject if BOTH position and rotation are too similar
            if tcp_dist < self._min_tcp_distance_m and tcp_rot < self._min_tcp_rotation_rad:
                return False

        # Reprojection error gate (RGB quality check)
        if reprojection_error_px > self.max_reprojection_error_px:
            self._rmse_reject_count += 1
            self.get_logger().warn(
                f"Reprojection error rejected: {reprojection_error_px:.2f}px > {self.max_reprojection_error_px:.2f}px "
                f"({self._rmse_reject_count}/{self.reprojection_reject_limit})"
            )
            return False

        host_stamp = float(host_stamp_override) if host_stamp_override is not None else time.time()
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        self.w.writerow([
            f"{host_stamp:.9f}",
            f"{obj_stamp:.9f}",
            tcp6[0], tcp6[1], tcp6[2], tcp6[3], tcp6[4], tcp6[5],
            p.x, p.y, p.z, q.x, q.y, q.z, q.w
        ])
        self.f.flush()

        # Update state
        self.lock_ref_tcp = tcp6
        self.armed = False
        self._rmse_reject_count = 0
        self.logged_count += 1
        self._last_logged_obj_stamp_us = obj_stamp_us
        self._last_logged_tcp = tcp6

        self.get_logger().info(f"Logged #{self.logged_count} (Reprojection Error: {reprojection_error_px:.2f}px)")
        self._publish_status(obj_stamp_sec=obj_stamp)
        return True

    def destroy_node(self):
        try:
            self.rtde_r.disconnect()
        except Exception:
            pass
        try:
            self.f.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HandEyeLogger()
    try:
        executor = rclpy.executors.MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

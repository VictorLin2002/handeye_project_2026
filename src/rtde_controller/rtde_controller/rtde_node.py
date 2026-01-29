import math
import rclpy

from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse

from rtde_controller_interfaces.action import MoveToPose
from .RTDE_utils import RTDEManager


class RTDEActionNode(Node):
    """
    RTDE-based motion server:
    - Accepts UR-style TCP pose [x, y, z, rx, ry, rz]
    - Workspace protection (3/4 circle, radial limits, min z)
    - Exposes MoveToPose action
    """

    def __init__(self):
        super().__init__("rtde_action_node")

        # Workspace config (meters, base frame)
        self.declare_parameter("ws.min_r", 0.20)
        self.declare_parameter("ws.max_r", 0.75)
        self.declare_parameter("ws.min_z", 0.20)
        self.declare_parameter("ws.forbidden_center_deg", 90.0)
        self.declare_parameter("ws.forbidden_width_deg", 90.0)

        self.ws = {
            "min_r": self.get_parameter("ws.min_r").value,
            "max_r": self.get_parameter("ws.max_r").value,
            "min_z": self.get_parameter("ws.min_z").value,
            "forbidden_center_deg": self.get_parameter("ws.forbidden_center_deg").value,
            "forbidden_width_deg": self.get_parameter("ws.forbidden_width_deg").value,
        }

        self.get_logger().info(
            "Safe workspace:\n"
            f"  r in [{self.ws['min_r']}, {self.ws['max_r']}] m\n"
            f"  z >= {self.ws['min_z']} m\n"
            f"  forbidden sector: center={self.ws['forbidden_center_deg']} deg, "
            f"width={self.ws['forbidden_width_deg']} deg (around +Y)"
        )

        # RTDE connection
        robot_ip = self.declare_parameter("robot_ip", "192.168.0.200").value

        self.rtde_mgr = RTDEManager(
            robot_ip=robot_ip,
            use_receive=False,
            use_control=True,
            use_io=False,
        )

        if not self.rtde_mgr.connect():
            self.get_logger().error("RTDE connect() failed; motions will be rejected.")
            self.rtde_c = None
        else:
            self.get_logger().info(f"RTDE connected to {robot_ip}")
            self.rtde_c = self.rtde_mgr.get_interface("control")

        # Action server: MoveToPose
        self._action_server = ActionServer(
            self,
            MoveToPose,
            "move_to_pose",
            execute_callback=self.execute_cb,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb,
        )

    # ------------------------------------------------------------------
    # Goal validation at reception
    # ------------------------------------------------------------------
    def goal_cb(self, goal_request: MoveToPose.Goal):
        if self.rtde_c is None:
            self.get_logger().warn("Rejecting goal: no RTDE control connection.")
            return GoalResponse.REJECT

        ok, reason = self.check_workspace(goal_request.x, goal_request.y, goal_request.z)
        if not ok:
            self.get_logger().warn(f"Rejecting goal (workspace): {reason}")
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    # ------------------------------------------------------------------
    # Cancel handling
    # ------------------------------------------------------------------
    def cancel_cb(self, goal_handle):
        # Synchronous moveL: no clean cancel support in this basic version.
        self.get_logger().warn("Cancel requested, but synchronous moveL cannot be cancelled.")
        return CancelResponse.REJECT

    # ------------------------------------------------------------------
    # Execute goal
    # ------------------------------------------------------------------
    # NOTE: Duplicate goal_cb method removed (was redundant with lines 75-85)
    # The original goal_cb above includes RTDE connection check which is important
    # ------------------------------------------------------------------

    async def execute_cb(self, goal_handle):
        """Execute the motion goal."""
        goal = goal_handle.request
        
        # Build TCP vector [x, y, z, rx, ry, rz]
        tcp = [goal.x, goal.y, goal.z, goal.rx, goal.ry, goal.rz]
        
        self.get_logger().info(f"Moving to TCP: {tcp}")
        
        # Send feedback
        feedback = MoveToPose.Feedback()
        feedback.progress = 0.0
        feedback.status = "Starting motion"
        goal_handle.publish_feedback(feedback)
        
        # Execute motion via RTDE
        result = MoveToPose.Result()
        if self.rtde_c is not None:
            try:
                self.get_logger().info(f"Calling moveL with tcp={tcp}, speed={goal.speed}, acc={goal.acc}")
                success = self.rtde_c.moveL(tcp, goal.speed, goal.acc, False)
                self.get_logger().info(f"moveL returned: {success}")
                result.success = bool(success)
                result.message = "Motion completed" if success else "Motion failed - check robot state and protective stop"
                
                feedback.progress = 1.0
                feedback.status = "Completed"
                goal_handle.publish_feedback(feedback)
                
            except Exception as e:
                result.success = False
                result.message = f"RTDE error: {str(e)}"
                self.get_logger().error(f"RTDE exception: {e}")
                
        else:
            result.success = False
            result.message = "RTDE not connected"
        
        if result.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
            
        return result

    # ------------------------------------------------------------------
    # Workspace check:
    #   - z >= min_z
    #   - min_r <= sqrt(x^2 + y^2) <= max_r
    #   - outside forbidden 1/4 circle around +Y (keep 3/4 circle facing -Y)
    # ------------------------------------------------------------------
    def check_workspace(self, x: float, y: float, z: float):
        if z < self.ws["min_z"]:
            return False, f"z {z:.3f} < {self.ws['min_z']:.3f}"

        r = math.hypot(x, y)
        if r < self.ws["min_r"] or r > self.ws["max_r"]:
            return False, f"r {r:.3f} not in [{self.ws['min_r']:.3f}, {self.ws['max_r']:.3f}]"

        theta = math.degrees(math.atan2(y, x))
        if theta < 0.0:
            theta += 360.0

        center = self.ws["forbidden_center_deg"]
        half_w = self.ws["forbidden_width_deg"] * 0.5
        diff = abs((theta - center + 180.0) % 360.0 - 180.0)

        if diff <= half_w:
            return False, f"angle {theta:.1f} deg inside forbidden sector"

        return True, "OK"

    # ------------------------------------------------------------------
    # Cleanup
    # ------------------------------------------------------------------
    def destroy_node(self):
        try:
            if self.rtde_mgr:
                self.rtde_mgr.disconnect()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RTDEActionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

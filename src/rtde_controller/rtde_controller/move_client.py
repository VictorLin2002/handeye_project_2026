import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from rtde_controller_interfaces.action import MoveToPose


class MoveClient(Node):
    """
    Simple test client for MoveToPose action (UR pose).
    """

    def __init__(self):
        super().__init__("move_client")
        self._client = ActionClient(self, MoveToPose, "move_to_pose")

    def send_goal_and_wait(self):
        goal = MoveToPose.Goal()

        # Example target:
        # - In safe 3/4 circle (towards -Y)
        # - UR axis-angle orientation: pointing down example (adjust for your setup)
        goal.x = -0.110
        goal.y = -0.4877
        goal.z = 0.4323
        goal.rx = 1.202
        goal.ry = 2.897
        goal.rz = -0.00

        goal.speed = 0.25
        goal.acc = 0.5

        self._client.wait_for_server()
        send_future = self._client.send_goal_async(
            goal,
            feedback_callback=self.feedback_cb,
        )
        rclpy.spin_until_future_complete(self, send_future)

        goal_handle = send_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        self.get_logger().info(
            f"Result: success={result.success}, message={result.message}"
        )

    def feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(
            f"Feedback: progress={fb.progress:.2f}, status={fb.status}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = MoveClient()
    node.send_goal_and_wait()
    node.destroy_node()
    rclpy.shutdown()

import rclpy
from rclpy.parameter import Parameter

from handeye_verify.verify_tag4_simple import Tag4ScrewToucher


def main() -> None:
    rclpy.init()
    node = Tag4ScrewToucher()
    node.set_parameters(
        [
            Parameter(
                "test_mode",
                Parameter.Type.STRING,
                "repeatability",
            )
        ]
    )

    try:
        node.get_logger().info("Running repeatability test...")
        ok = node.run_repeatability()
        if not ok:
            node.get_logger().error("Repeatability test failed.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

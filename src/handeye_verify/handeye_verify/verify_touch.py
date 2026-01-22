import rclpy

from handeye_verify.tag4_screw_toucher import Tag4ScrewToucher


def main() -> None:
    rclpy.init()
    node = Tag4ScrewToucher()

    try:
        node.get_logger().info("Running touch sequence...")
        ok = node.run_once_touch()
        if not ok:
            node.get_logger().error("Touch sequence failed.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

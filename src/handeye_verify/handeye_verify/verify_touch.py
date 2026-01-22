import time

import rclpy

from handeye_verify.tag4_screw_toucher import Tag4ScrewToucher


def main() -> None:
    rclpy.init()
    node = Tag4ScrewToucher()
    repeat = bool(node.get_parameter("repeat_touch").value)
    repeat_interval = float(node.get_parameter("repeat_interval_sec").value)

    try:
        cycle = 0
        while rclpy.ok():
            cycle += 1
            node.get_logger().info(f"Running touch sequence (cycle {cycle})...")
            ok = node.run_once_touch()
            if not ok:
                node.get_logger().error("Touch sequence failed.")
                break
            if not repeat:
                break
            if repeat_interval > 0.0:
                time.sleep(repeat_interval)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

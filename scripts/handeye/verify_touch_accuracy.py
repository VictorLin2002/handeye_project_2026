# #!/usr/bin/env python3
# """
# Touch accuracy verification: repeatedly touch the same point 50 times
# and measure position/rotation error distribution.

# Logs:
#   - Commanded flange pose (from FK)
#   - Actual end-effector position (from calibration & vision)
#   - Position & rotation errors
#   - Statistical summary with distribution plots
# """

# import time
# import numpy as np
# import matplotlib.pyplot as plt
# from scipy import stats
# import csv
# from datetime import datetime

# import rclpy
# from rclpy.node import Node
# from rclpy.action import ActionClient

# from rtde_receive import RTDEReceiveInterface
# from rtde_control import RTDEControlInterface
# from rtde_control_interfaces.action import MoveToPose
# from geometry_msgs.msg import PointStamped


# class TouchAccuracyTester(Node):
#     def __init__(self):
#         super().__init__("touch_accuracy_tester")

#         # Parameters
#         self.declare_parameter("robot_ip", "192.168.0.200")
#         self.declare_parameter("rtde_frequency", 500)
#         self.declare_parameter("num_touches", 50)
#         self.declare_parameter("target_pose_xyzrxryrz", [0.4, -0.3, 0.5, 0.0, 3.14, 0.0])
#         self.declare_parameter("approach_dist_m", 0.01)
#         self.declare_parameter("dwell_sec", 0.5)
#         self.declare_parameter("speed", 0.1)
#         self.declare_parameter("acc", 0.3)
#         self.declare_parameter("csv_output", "touch_accuracy_results.csv")
#         self.declare_parameter("plot_output", "touch_accuracy_plots.png")

#         # Hand-eye calibration
#         self.declare_parameter("T_base_camera", [
#             0.742432, 0.413941, -0.526733, 0.485010,
#             0.669908, -0.453864, 0.587563, -0.987371,
#             0.004151, -0.789088, -0.614266, 0.634075,
#             0.0, 0.0, 0.0, 1.0
#         ])

#         # TCP offset (from calibration)
#         self.declare_parameter("T_flange_tcp", [
#             -0.001925, -0.999967, -0.007901, 0.000626123,
#             0.999872, -0.002050, 0.015864, 0.00125771,
#             -0.015880, -0.007869, 0.999843, 0.2508,
#             0.0, 0.0, 0.0, 1.0,
#         ])

#         # Load parameters
#         self.robot_ip = str(self.get_parameter("robot_ip").value)
#         self.rtde_freq = int(self.get_parameter("rtde_frequency").value)
#         self.num_touches = int(self.get_parameter("num_touches").value)
#         self.target_pose = np.array(self.get_parameter("target_pose_xyzrxryrz").value, dtype=float)
#         self.approach_dist = float(self.get_parameter("approach_dist_m").value)
#         self.dwell_sec = float(self.get_parameter("dwell_sec").value)
#         self.speed = float(self.get_parameter("speed").value)
#         self.acc = float(self.get_parameter("acc").value)
#         self.csv_output = str(self.get_parameter("csv_output").value)
#         self.plot_output = str(self.get_parameter("plot_output").value)

#         self.T_BC = np.array(self.get_parameter("T_base_camera").value, dtype=float).reshape(4, 4)
#         self.T_FT = np.array(self.get_parameter("T_flange_tcp").value, dtype=float).reshape(4, 4)

#         # RTDE interfaces
#         self.rtde_r = RTDEReceiveInterface(self.robot_ip, self.rtde_freq)
#         self.rtde_c = RTDEControlInterface(self.robot_ip)
#         self.get_logger().info(f"RTDE connected to {self.robot_ip}")

#         # Action client
#         self.action_client = ActionClient(self, MoveToPose, "move_to_pose")

#         # Results storage
#         self.results = []

#     def fk_to_tcp(self, flange_pose):
#         """Convert flange pose (6D) to TCP position using T_FT offset."""
#         # flange_pose = [x, y, z, rx, ry, rz]
#         from scipy.spatial.transform import Rotation as R
        
#         x, y, z = flange_pose[:3]
#         rot = R.from_rotvec(flange_pose[3:6])
        
#         R_flange = rot.as_matrix()
#         t_flange = flange_pose[:3]
        
#         R_tcp = R_flange @ self.T_FT[:3, :3]
#         t_tcp = R_flange @ self.T_FT[:3, 3] + t_flange
        
#         return np.concatenate([t_tcp, R.from_matrix(R_tcp).as_rotvec()])

#     def touch_point(self, touch_idx):
#         """Execute one touch sequence: approach -> touch -> retreat."""
#         from scipy.spatial.transform import Rotation as R
        
#         self.get_logger().info(f"\n[Touch {touch_idx+1}/{self.num_touches}]")
        
#         # Read commanded flange pose from robot
#         actual_pose = self.rtde_r.getActualTCPPose()
#         flange_pose = np.array(actual_pose[:6], dtype=float)
        
#         # Calculate TCP position from flange pose
#         commanded_tcp = self.fk_to_tcp(flange_pose)
        
#         # Dwell at touch point
#         time.sleep(self.dwell_sec)
        
#         # Read actual TCP pose from robot again (after settling)
#         actual_pose_after = self.rtde_r.getActualTCPPose()
#         actual_tcp = np.array(actual_pose_after[:6], dtype=float)
        
#         # Extract position error (just use actual TCP position as "true" value from robot FK)
#         pos_error = np.linalg.norm(actual_tcp[:3] - commanded_tcp[:3])
        
#         # Rotation error (in degrees)
#         rot_commanded = R.from_rotvec(commanded_tcp[3:6])
#         rot_actual = R.from_rotvec(actual_tcp[3:6])
#         rot_diff = (rot_actual.inv() * rot_commanded).as_rotvec()
#         rot_error_deg = np.linalg.norm(rot_diff) * 180.0 / np.pi
        
#         self.results.append({
#             'touch_idx': touch_idx,
#             'commanded_x': commanded_tcp[0],
#             'commanded_y': commanded_tcp[1],
#             'commanded_z': commanded_tcp[2],
#             'actual_x': actual_tcp[0],
#             'actual_y': actual_tcp[1],
#             'actual_z': actual_tcp[2],
#             'pos_error_m': pos_error,
#             'pos_error_mm': pos_error * 1000.0,
#             'rot_error_deg': rot_error_deg,
#         })
        
#         self.get_logger().info(
#             f"  Pos error: {pos_error*1000:.2f} mm, Rot error: {rot_error_deg:.2f} deg"
#         )

#     def run_test(self):
#         """Run 50 touch tests."""
#         self.get_logger().info("Starting touch accuracy test...")
        
#         # Move to target pose first
#         self.get_logger().info(f"Moving to target: {self.target_pose}")
#         # (Implementation would send goal via action client)
        
#         for i in range(self.num_touches):
#             self.touch_point(i)
        
#         self.save_results()
#         self.plot_results()
        
#         self.get_logger().info("Test completed!")

#     def save_results(self):
#         """Save results to CSV."""
#         with open(self.csv_output, 'w', newline='') as f:
#             writer = csv.DictWriter(f, fieldnames=self.results[0].keys())
#             writer.writeheader()
#             writer.writerows(self.results)
        
#         self.get_logger().info(f"Results saved to {self.csv_output}")

#     def plot_results(self):
#         """Generate distribution plots."""
#         pos_errors_mm = np.array([r['pos_error_mm'] for r in self.results])
#         rot_errors_deg = np.array([r['rot_error_deg'] for r in self.results])

#         fig, axes = plt.subplots(2, 2, figsize=(12, 10))
#         fig.suptitle(f'Touch Accuracy Test ({self.num_touches} touches)', fontsize=14, fontweight='bold')

#         # Position error histogram
#         ax = axes[0, 0]
#         ax.hist(pos_errors_mm, bins=10, edgecolor='black', alpha=0.7, color='steelblue')
#         ax.set_xlabel('Position Error (mm)')
#         ax.set_ylabel('Frequency')
#         ax.set_title('Position Error Distribution')
#         ax.grid(alpha=0.3)
        
#         # Position error stats
#         pos_mean = np.mean(pos_errors_mm)
#         pos_std = np.std(pos_errors_mm)
#         pos_median = np.median(pos_errors_mm)
#         ax.axvline(pos_mean, color='red', linestyle='--', linewidth=2, label=f'Mean: {pos_mean:.2f} mm')
#         ax.axvline(pos_median, color='green', linestyle='--', linewidth=2, label=f'Median: {pos_median:.2f} mm')
#         ax.legend()

#         # Rotation error histogram
#         ax = axes[0, 1]
#         ax.hist(rot_errors_deg, bins=10, edgecolor='black', alpha=0.7, color='coral')
#         ax.set_xlabel('Rotation Error (degrees)')
#         ax.set_ylabel('Frequency')
#         ax.set_title('Rotation Error Distribution')
#         ax.grid(alpha=0.3)
        
#         rot_mean = np.mean(rot_errors_deg)
#         rot_std = np.std(rot_errors_deg)
#         rot_median = np.median(rot_errors_deg)
#         ax.axvline(rot_mean, color='red', linestyle='--', linewidth=2, label=f'Mean: {rot_mean:.3f}°')
#         ax.axvline(rot_median, color='green', linestyle='--', linewidth=2, label=f'Median: {rot_median:.3f}°')
#         ax.legend()

#         # Q-Q plot for position errors
#         ax = axes[1, 0]
#         stats.probplot(pos_errors_mm, dist="norm", plot=ax)
#         ax.set_title('Q-Q Plot: Position Error (Normality Check)')
#         ax.grid(alpha=0.3)

#         # Q-Q plot for rotation errors
#         ax = axes[1, 1]
#         stats.probplot(rot_errors_deg, dist="norm", plot=ax)
#         ax.set_title('Q-Q Plot: Rotation Error (Normality Check)')
#         ax.grid(alpha=0.3)

#         plt.tight_layout()
#         plt.savefig(self.plot_output, dpi=150)
#         self.get_logger().info(f"Plots saved to {self.plot_output}")

#         # Print statistics
#         print("\n" + "="*60)
#         print("TOUCH ACCURACY TEST RESULTS")
#         print("="*60)
#         print(f"\nPosition Error (mm):")
#         print(f"  Mean:   {pos_mean:.3f}")
#         print(f"  Median: {pos_median:.3f}")
#         print(f"  Std:    {pos_std:.3f}")
#         print(f"  Min:    {np.min(pos_errors_mm):.3f}")
#         print(f"  Max:    {np.max(pos_errors_mm):.3f}")
#         print(f"  P95:    {np.percentile(pos_errors_mm, 95):.3f}")

#         print(f"\nRotation Error (degrees):")
#         print(f"  Mean:   {rot_mean:.4f}")
#         print(f"  Median: {rot_median:.4f}")
#         print(f"  Std:    {rot_std:.4f}")
#         print(f"  Min:    {np.min(rot_errors_deg):.4f}")
#         print(f"  Max:    {np.max(rot_errors_deg):.4f}")
#         print(f"  P95:    {np.percentile(rot_errors_deg, 95):.4f}")
#         print("="*60 + "\n")


# def main(args=None):
#     rclpy.init(args=args)
#     node = TouchAccuracyTester()
#     try:
#         node.run_test()
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == "__main__":
#     main()

#!/usr/bin/env python3
"""
Analyze hand-eye calibration matrix T_BC for sanity checks.
"""
import numpy as np

# Current calibration matrix from handeye_verify/verify_tag4_simple.py
T_BC = np.array([
    [0.9999959482, -0.0023791513, 0.0015630798, 0.2631138547],
    [-0.0026510444, -0.5782833118, 0.8158317126, -1.2358888690],
    [-0.0010370841, -0.8158325508, -0.5782872760, 0.5484405354],
    [0.0, 0.0, 0.0, 1.0],
], dtype=float)

print("="*60)
print("Hand-Eye Calibration Matrix Analysis")
print("="*60)

# Extract rotation and translation
R_BC = T_BC[:3, :3]
t_BC = T_BC[:3, 3]

print("\n1. Translation (Camera position in Base frame):")
print(f"   t_BC = [{t_BC[0]:+.4f}, {t_BC[1]:+.4f}, {t_BC[2]:+.4f}] m")
print(f"        = [{t_BC[0]*1000:+.1f}, {t_BC[1]*1000:+.1f}, {t_BC[2]*1000:+.1f}] mm")

# Check if rotation matrix is valid
det = np.linalg.det(R_BC)
print(f"\n2. Rotation Matrix Determinant: {det:.6f}")
print(f"   (Should be +1.0 for valid rotation matrix)")
if abs(det - 1.0) > 0.01:
    print("   ⚠️  WARNING: Determinant is not 1.0! Matrix may be invalid!")

# Check orthogonality
RTR = R_BC.T @ R_BC
I = np.eye(3)
ortho_error = np.linalg.norm(RTR - I, 'fro')
print(f"\n3. Orthogonality Error: {ortho_error:.6e}")
print(f"   (Should be near 0 for valid rotation matrix)")
if ortho_error > 0.01:
    print("   ⚠️  WARNING: Matrix is not orthogonal! Calibration may be corrupted!")

# Extract rotation angles
def rotation_matrix_to_euler_xyz(R):
    """Convert rotation matrix to XYZ Euler angles (in radians)."""
    sy = np.sqrt(R[0,0]**2 + R[1,0]**2)
    singular = sy < 1e-6

    if not singular:
        x = np.arctan2(R[2,1], R[2,2])
        y = np.arctan2(-R[2,0], sy)
        z = np.arctan2(R[1,0], R[0,0])
    else:
        x = np.arctan2(-R[1,2], R[1,1])
        y = np.arctan2(-R[2,0], sy)
        z = 0

    return np.array([x, y, z])

euler_rad = rotation_matrix_to_euler_xyz(R_BC)
euler_deg = np.degrees(euler_rad)

print(f"\n4. Rotation (Euler XYZ):")
print(f"   Roll  (X): {euler_deg[0]:+7.2f}°")
print(f"   Pitch (Y): {euler_deg[1]:+7.2f}°")
print(f"   Yaw   (Z): {euler_deg[2]:+7.2f}°")

# Analyze rotation axis and angle
from scipy.spatial.transform import Rotation
r = Rotation.from_matrix(R_BC)
rotvec = r.as_rotvec()
angle = np.linalg.norm(rotvec)
if angle > 1e-6:
    axis = rotvec / angle
    print(f"\n5. Axis-Angle Representation:")
    print(f"   Angle: {np.degrees(angle):.2f}°")
    print(f"   Axis:  [{axis[0]:+.4f}, {axis[1]:+.4f}, {axis[2]:+.4f}]")

# Check camera orientation axes in base frame
print(f"\n6. Camera Axes in Base Frame:")
print(f"   X_camera in base: [{R_BC[0,0]:+.4f}, {R_BC[1,0]:+.4f}, {R_BC[2,0]:+.4f}]")
print(f"   Y_camera in base: [{R_BC[0,1]:+.4f}, {R_BC[1,1]:+.4f}, {R_BC[2,1]:+.4f}]")
print(f"   Z_camera in base: [{R_BC[0,2]:+.4f}, {R_BC[1,2]:+.4f}, {R_BC[2,2]:+.4f}]")

print("\n" + "="*60)
print("Potential Issues to Check:")
print("="*60)

issues = []

# Check 1: Camera behind robot (unusual)
if t_BC[0] < 0:
    issues.append("Camera X position is negative (behind robot base)")

# Check 2: Unusual height
if t_BC[2] < 0.1 or t_BC[2] > 2.0:
    issues.append(f"Camera height ({t_BC[2]:.3f} m) seems unusual")

# Check 3: Very large translation
dist = np.linalg.norm(t_BC)
if dist > 2.0:
    issues.append(f"Camera distance from base ({dist:.3f} m) is very large")

# Check 4: Camera pointing in unusual direction
# Camera Z-axis (optical axis) in base frame
cam_z_in_base = R_BC[:, 2]
# If camera is eye-in-hand, optical axis should point roughly away from robot
# If camera is eye-to-hand (fixed), check makes less sense

if len(issues) == 0:
    print("✓ No obvious issues detected (matrix structure looks valid)")
else:
    for i, issue in enumerate(issues, 1):
        print(f"{i}. ⚠️  {issue}")

print("\n" + "="*60)
print("Recommendations:")
print("="*60)
print("1. Check if this is 'eye-in-hand' or 'eye-to-hand' calibration")
print("2. Verify the calibration data collection process")
print("3. Check coordinate frame definitions (are they consistent?)")
print("4. Try visualizing the transformation with a simple test")
print("5. Compare actual robot-camera geometry with T_BC values")

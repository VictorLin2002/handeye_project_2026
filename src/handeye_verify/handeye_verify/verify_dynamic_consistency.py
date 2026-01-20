import argparse
from dataclasses import dataclass
from typing import List, Tuple

import numpy as np


@dataclass
class Pose6D:
    x: float
    y: float
    z: float
    rx: float
    ry: float
    rz: float


@dataclass
class PoseQuat:
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float


def rotvec_to_matrix(rotvec: np.ndarray) -> np.ndarray:
    angle = np.linalg.norm(rotvec)
    if angle < 1e-12:
        return np.eye(3)
    axis = rotvec / angle
    kx, ky, kz = axis
    k = np.array(
        [
            [0.0, -kz, ky],
            [kz, 0.0, -kx],
            [-ky, kx, 0.0],
        ],
        dtype=float,
    )
    return np.eye(3) + np.sin(angle) * k + (1.0 - np.cos(angle)) * (k @ k)


def quat_to_matrix(q: np.ndarray) -> np.ndarray:
    qx, qy, qz, qw = q
    n = np.linalg.norm(q)
    if n < 1e-12:
        return np.eye(3)
    qx, qy, qz, qw = q / n
    return np.array(
        [
            [1 - 2 * (qy**2 + qz**2), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx**2 + qz**2), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx**2 + qy**2)],
        ],
        dtype=float,
    )


def make_transform_from_pose6(pose: Pose6D) -> np.ndarray:
    rotvec = np.array([pose.rx, pose.ry, pose.rz], dtype=float)
    R = rotvec_to_matrix(rotvec)
    t = np.array([pose.x, pose.y, pose.z], dtype=float)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def make_transform_from_pose_quat(pose: PoseQuat) -> np.ndarray:
    R = quat_to_matrix(np.array([pose.qx, pose.qy, pose.qz, pose.qw], dtype=float))
    t = np.array([pose.x, pose.y, pose.z], dtype=float)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def matrix_to_euler_xyz(R: np.ndarray) -> np.ndarray:
    sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    singular = sy < 1e-6
    if not singular:
        x = np.arctan2(R[2, 1], R[2, 2])
        y = np.arctan2(-R[2, 0], sy)
        z = np.arctan2(R[1, 0], R[0, 0])
    else:
        x = np.arctan2(-R[1, 2], R[1, 1])
        y = np.arctan2(-R[2, 0], sy)
        z = 0.0
    return np.array([x, y, z], dtype=float)


def load_samples(csv_path: str) -> List[Tuple[Pose6D, PoseQuat]]:
    import csv

    samples = []
    skipped = 0
    with open(csv_path, newline="") as csvfile:
        reader = csv.DictReader(csvfile)
        for index, row in enumerate(reader, start=2):
            try:
                base_tcp = Pose6D(
                    float(row["B_tcp_x"]),
                    float(row["B_tcp_y"]),
                    float(row["B_tcp_z"]),
                    float(row["B_tcp_rx"]),
                    float(row["B_tcp_ry"]),
                    float(row["B_tcp_rz"]),
                )
                cam_tag = PoseQuat(
                    float(row["C_p_O_x"]),
                    float(row["C_p_O_y"]),
                    float(row["C_p_O_z"]),
                    float(row["C_q_O_x"]),
                    float(row["C_q_O_y"]),
                    float(row["C_q_O_z"]),
                    float(row["C_q_O_w"]),
                )
            except (TypeError, ValueError) as exc:
                skipped += 1
                print(f"Skipping invalid row {index}: {exc}")
                continue
            samples.append((base_tcp, cam_tag))
    if skipped:
        print(f"Skipped {skipped} invalid rows while parsing CSV.")
    return samples


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Dynamic consistency test: evaluate flange-to-tag rotation stability."
    )
    parser.add_argument("--csv", required=True, help="CSV file with handeye samples")
    parser.add_argument(
        "--tbc",
        required=True,
        nargs=7,
        type=float,
        metavar=("X", "Y", "Z", "QX", "QY", "QZ", "QW"),
        help="Base->Camera transform (translation meters + quaternion)",
    )
    args = parser.parse_args()

    tbc_pose = PoseQuat(
        args.tbc[0],
        args.tbc[1],
        args.tbc[2],
        args.tbc[3],
        args.tbc[4],
        args.tbc[5],
        args.tbc[6],
    )

    samples = load_samples(args.csv)
    if not samples:
        raise SystemExit("No samples found in CSV.")

    T_base_camera = make_transform_from_pose_quat(tbc_pose)

    eulers_deg = []
    for base_tcp, cam_tag in samples:
        T_base_flange = make_transform_from_pose6(base_tcp)
        T_camera_tag = make_transform_from_pose_quat(cam_tag)

        T_flange_tag = np.linalg.inv(T_base_flange) @ T_base_camera @ T_camera_tag
        euler_rad = matrix_to_euler_xyz(T_flange_tag[:3, :3])
        eulers_deg.append(np.degrees(euler_rad))

    eulers_deg = np.array(eulers_deg)
    std_deg = np.std(eulers_deg, axis=0, ddof=1)

    print("Dynamic consistency test (flange->tag rotation stability)")
    print(f"Samples: {len(samples)}")
    print("Euler XYZ std dev (deg):")
    print(f"  X: {std_deg[0]:.3f} deg")
    print(f"  Y: {std_deg[1]:.3f} deg")
    print(f"  Z: {std_deg[2]:.3f} deg")
    max_std = float(np.max(std_deg))
    if max_std < 0.5:
        verdict = "Excellent"
    elif max_std < 1.0:
        verdict = "Acceptable"
    else:
        verdict = "Fail"
    print(f"Verdict: {verdict} (max std dev {max_std:.3f} deg)")


if __name__ == "__main__":
    main()

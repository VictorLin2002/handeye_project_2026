#!/usr/bin/env bash
# Generate calibration poses for Eye-to-Hand hand-eye calibration (camera fixed, target on TCP).
# Output CSV: x,y,z,rx,ry,rz  (UR rotvec)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/../_lib/env.sh"

OUT_DIR="${1:-${WS_ROOT}}"
CSV="${OUT_DIR}/calib_poses.csv"
mkdir -p "${OUT_DIR}"

python3 - <<'PY' > "${CSV}"
import math
import random

# -----------------------------
# Configuration
# -----------------------------
# Camera approximate position in Base frame (meters)
CAMERA_POS = dict(x=0.5148, y=-1.0007, z=0.6088)

# Position center in Base frame (meters). You said position is fine -> keep this as your pivot.
ORIGIN = dict(x=0.15, y=-0.455, z=0.400)

# Image-center orientation Rc (UR rotvec, radians) in Base frame
# IMPORTANT: set this to your "image center pose" orientation (Rc), not a random center.
RC_RV = (1.784, 1.29100, -0.0040)  # (rx, ry, rz)

# Plane grid half-span (meters): u,v in {-A,+A} or {-A,0,+A}
A = 0.12  # Increased for larger XY coverage
# Depth layers along n (meters): w in {-B,+B}
B = 0.045  # Increased for larger Z coverage

U_SET = [-A, 0.0, +A]     # 3 positions along t1
V_SET = [-A, +A]          # 2 positions along t2
W_SET = [-B, +B]          # 2 depth layers
# Total: 3 × 2 × 2 = 12 positions

# Gates
MIN_DIST_TO_CAM = 0.4
MAX_DIST_TO_CAM = 1.0
MAX_VIEW_ANGLE_DEG = 88.0  # loosened to allow steep viewing angles

# Excitations
SPIN_DEG = 60.0   # rotation around n (normal axis)
TILT_DEG = 50.0   # rotation around t1 (in-plane axis)
TILT2_DEG = 50.0  # rotation around t2 (second in-plane axis)

# Reproducibility
SEED = 42
random.seed(SEED)

# -----------------------------
# Math helpers
# -----------------------------
def dot(a, b):
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]

def norm(v):
    l = math.sqrt(dot(v, v))
    if l < 1e-12:
        return [0.0, 0.0, 0.0]
    return [v[0]/l, v[1]/l, v[2]/l]

def sub(a, b):
    return [a[0]-b[0], a[1]-b[1], a[2]-b[2]]

def add(a, b):
    return [a[0]+b[0], a[1]+b[1], a[2]+b[2]]

def scale(v, s):
    return [v[0]*s, v[1]*s, v[2]*s]

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

def cross(a, b):
    return [
        a[1]*b[2] - a[2]*b[1],
        a[2]*b[0] - a[0]*b[2],
        a[0]*b[1] - a[1]*b[0],
    ]

def distance(p1, p2):
    d = sub(p1, p2)
    return math.sqrt(dot(d, d))

def mat_mul(A, B):
    return [[
        A[i][0]*B[0][j] + A[i][1]*B[1][j] + A[i][2]*B[2][j]
        for j in range(3)
    ] for i in range(3)]

def rotvec_to_R(rx, ry, rz):
    th = math.sqrt(rx*rx + ry*ry + rz*rz)
    if th < 1e-12:
        return [[1,0,0],[0,1,0],[0,0,1]]
    kx, ky, kz = rx/th, ry/th, rz/th
    c, s = math.cos(th), math.sin(th)
    v = 1.0 - c
    return [
        [kx*kx*v + c,    kx*ky*v - kz*s, kx*kz*v + ky*s],
        [kx*ky*v + kz*s, ky*ky*v + c,    ky*kz*v - kx*s],
        [kx*kz*v - ky*s, ky*kz*v + kx*s, kz*kz*v + c],
    ]

def R_to_rotvec(R):
    tr = R[0][0] + R[1][1] + R[2][2]
    val = clamp((tr - 1.0) * 0.5, -1.0, 1.0)
    th = math.acos(val)
    if th < 1e-12:
        return [0.0, 0.0, 0.0]
    sx = R[2][1] - R[1][2]
    sy = R[0][2] - R[2][0]
    sz = R[1][0] - R[0][1]
    s = math.sqrt(sx*sx + sy*sy + sz*sz)
    if s < 1e-12:
        return [0.0, 0.0, 0.0]
    k = th / s
    return [k*sx, k*sy, k*sz]

def R_axis_angle(axis, angle_rad):
    # Rodrigues formula
    a = norm(axis)
    kx, ky, kz = a[0], a[1], a[2]
    c, s = math.cos(angle_rad), math.sin(angle_rad)
    v = 1.0 - c
    return [
        [kx*kx*v + c,     kx*ky*v - kz*s,  kx*kz*v + ky*s],
        [kx*ky*v + kz*s,  ky*ky*v + c,     ky*kz*v - kx*s],
        [kx*kz*v - ky*s,  ky*kz*v + kx*s,  kz*kz*v + c],
    ]

# -----------------------------
# Build basis from Rc and camera position
# -----------------------------
cam = [CAMERA_POS["x"], CAMERA_POS["y"], CAMERA_POS["z"]]
P0  = [ORIGIN["x"], ORIGIN["y"], ORIGIN["z"]]

Rc = rotvec_to_R(*RC_RV)

# Normal from Rc: board normal assumed to be -Z of Rc
n = [-Rc[0][2], -Rc[1][2], -Rc[2][2]]
n = norm(n)

# Use camera position to select the sign so the board faces the camera
# If n points away from the camera, flip it.
if dot(n, sub(P0, cam)) < 0.0:
    n = scale(n, -1.0)

# In-plane axes from Rc, projected to the plane orthogonal to n (numerical robustness)
x_axis = [Rc[0][0], Rc[1][0], Rc[2][0]]
t1 = sub(x_axis, scale(n, dot(x_axis, n)))
t1 = norm(t1)

# If degenerate, fall back to Rc y-axis
if math.sqrt(dot(t1, t1)) < 1e-6:
    y_axis = [Rc[0][1], Rc[1][1], Rc[2][1]]
    t1 = sub(y_axis, scale(n, dot(y_axis, n)))
    t1 = norm(t1)

t2 = norm(cross(n, t1))

max_view_rad = math.radians(MAX_VIEW_ANGLE_DEG)

def make_position(u, v, w):
    # p = P0 + u*t1 + v*t2 + w*n
    return add(add(add(P0, scale(t1, u)), scale(t2, v)), scale(n, w))

def pass_gates(pos, R):
    # Distance gate
    d = distance(pos, cam)
    if not (MIN_DIST_TO_CAM <= d <= MAX_DIST_TO_CAM):
        return False

    # View angle gate: board normal (-Z of R) vs camera ray (camera -> pos)
    ray = norm(sub(pos, cam))
    z_col = [R[0][2], R[1][2], R[2][2]]
    normal = [-z_col[0], -z_col[1], -z_col[2]]  # -Z
    ang = math.acos(clamp(dot(norm(normal), ray), -1.0, 1.0))
    return ang <= max_view_rad

def emit_pose(pos, R):
    if not pass_gates(pos, R):
        return None
    rv = R_to_rotvec(R)
    return (pos[0], pos[1], pos[2], rv[0], rv[1], rv[2])

# -----------------------------
# Generate ~40 poses
# -----------------------------
print("x,y,z,rx,ry,rz")

poses = []
seen = set()

def add_pose(out):
    if out is None:
        return
    key = tuple(round(x, 6) for x in out)
    if key in seen:
        return
    poses.append(out)
    seen.add(key)

# Base positions: 3x3x2 = 18, all with Rc
grid = []
for w in W_SET:
    for v in V_SET:
        for u in U_SET:
            pos = make_position(u, v, w)
            grid.append((u, v, w, pos))

# Snake ordering to reduce travel
def snake_key(item):
    u, v, w, _ = item
    wi = 0 if w < 0 else 1
    vi = { -A:0, 0.0:1, +A:2 }[v]
    ui = { -A:0, 0.0:1, +A:2 }[u]
    if (wi + vi) % 2 == 1:
        ui = 2 - ui
    return (wi, vi, ui)

grid.sort(key=snake_key)

pos_map = {(u, v, w): pos for (u, v, w, pos) in grid}

# Generate rotation variations around different axes
R_spin_p = R_axis_angle(n, math.radians(+SPIN_DEG))
R_spin_m = R_axis_angle(n, math.radians(-SPIN_DEG))
R_tilt1_p = R_axis_angle(t1, math.radians(+TILT_DEG))
R_tilt1_m = R_axis_angle(t1, math.radians(-TILT_DEG))
R_tilt2_p = R_axis_angle(t2, math.radians(+TILT2_DEG))
R_tilt2_m = R_axis_angle(t2, math.radians(-TILT2_DEG))

# For each grid position, generate 8 orientations with comprehensive rotations
for i, (_, _, _, pos) in enumerate(grid):
    # 1) Base orientation (Rc) - always include for all positions
    add_pose(emit_pose(pos, Rc))

    # 2) Spin rotations around normal (n axis) - both directions
    add_pose(emit_pose(pos, mat_mul(Rc, R_spin_p)))
    add_pose(emit_pose(pos, mat_mul(Rc, R_spin_m)))

    # 3) Tilt rotations around t1 - both directions
    add_pose(emit_pose(pos, mat_mul(Rc, R_tilt1_p)))
    add_pose(emit_pose(pos, mat_mul(Rc, R_tilt1_m)))

    # 4) Tilt rotations around t2 - both directions
    add_pose(emit_pose(pos, mat_mul(Rc, R_tilt2_p)))
    add_pose(emit_pose(pos, mat_mul(Rc, R_tilt2_m)))

    # 5) Combined rotation for extra diversity
    add_pose(emit_pose(pos, mat_mul(Rc, mat_mul(R_spin_p, R_tilt1_p))))

for (x, y, z, rx, ry, rz) in poses:
    print(f"{x:.6f},{y:.6f},{z:.6f},{rx:.6f},{ry:.6f},{rz:.6f}")
PY



echo "Wrote: ${CSV}"

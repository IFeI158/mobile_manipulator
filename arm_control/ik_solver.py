"""
ik_solver.py — Inverse Kinematics module for 4-DOF robot arm

Computes servo angles (s0~s3) from a 3D target position (camera frame).
All angles are clamped to [0, 180] degrees.

Robot parameters are based on physical measurements of the capstone robot arm.
"""

import math

# ── Link lengths (cm) ──────────────────────────────────────────────────────────
L1 = 20.404   # Shoulder → Elbow
L2 = 17.617   # Elbow    → Wrist
L3 = 4.65     # Wrist    → End-effector

# ── Base offset ────────────────────────────────────────────────────────────────
BASE_Z_OFFSET = 7.8   # Base → first joint height (cm)

# ── Initial (home) pose angles (degrees) ──────────────────────────────────────
INIT_S1 = 130.0
INIT_S2 = 0.0
INIT_S3 = 130.0

# ── Empirical X correction (cm) ───────────────────────────────────────────────
# Compensates for mechanical offset observed during calibration.
X_CORRECTION = -8.0

# ── Pre-computed camera offset at home pose ───────────────────────────────────
def _compute_initial_cam_offset():
    """
    Calculates the end-effector position in the base frame at the home pose.
    Used to convert camera-frame coordinates to base-frame coordinates.
    """
    alpha1 = math.radians(180.0 - INIT_S1)
    theta2 = math.radians(INIT_S1 + INIT_S2 - 90.0)
    theta3 = theta2 - math.radians(INIT_S3 - 90.0)

    cam_x = (-L1 * math.cos(alpha1)
             + L2 * math.cos(theta2)
             + L3 * math.cos(theta3))
    cam_y = 0.0
    cam_z = (BASE_Z_OFFSET
             + L1 * math.sin(alpha1)
             + L2 * math.sin(theta2)
             + L3 * math.sin(theta3))
    return cam_x, cam_y, cam_z


CAM_OFFSET_X, CAM_OFFSET_Y, CAM_OFFSET_Z = _compute_initial_cam_offset()


# ── Utility ────────────────────────────────────────────────────────────────────
def clamp(val, min_val=0.0, max_val=180.0):
    """Clamp val to [min_val, max_val]."""
    return max(min_val, min(max_val, val))


def _safe_acos(x):
    """acos with numeric clamp to avoid domain errors."""
    return math.acos(max(-1.0, min(1.0, x)))


# ── Main IK solver ─────────────────────────────────────────────────────────────
def compute_servo_angles(x_cam, y_cam, z_cam):
    """
    Convert camera-frame 3D coordinates (cm) to servo angles.

    Parameters
    ----------
    x_cam, y_cam, z_cam : float
        Target position in camera frame (centimetres).

    Returns
    -------
    (s0, s1, s2, s3) : tuple of int
        Servo angles in degrees, each clamped to [0, 180].

    Notes
    -----
    - s0 : base yaw
    - s1 : shoulder (link 1)
    - s2 : elbow    (link 2)
    - s3 : wrist    (link 3)

    If the target is outside the arm's reachable workspace,
    the target is scaled to the nearest reachable point (no exception raised).
    """
    # 1. Camera → base frame
    x_b = x_cam + CAM_OFFSET_X + X_CORRECTION
    y_b = y_cam + CAM_OFFSET_Y
    z_b = z_cam + CAM_OFFSET_Z

    # 2. Base yaw (s0)
    s0 = (math.degrees(math.atan2(y_b, x_b)) + 90.0
          if abs(y_b) >= 1e-5 else 90.0)
    s0 = clamp(s0)

    # 3. Planar distance with wrist offset removed
    r  = math.hypot(x_b, y_b)
    r2 = r - L3
    z2 = z_b - BASE_Z_OFFSET
    d  = math.hypot(r2, z2)

    # 4. Clamp to reachable workspace (scale instead of raising)
    MAX_REACH = L1 + L2 - 0.001
    MIN_REACH = abs(L1 - L2) + 0.001
    if d > MAX_REACH or d < MIN_REACH:
        scale = (MAX_REACH if d > MAX_REACH else MIN_REACH) / d
        r2 *= scale
        z2 *= scale
        d   = math.hypot(r2, z2)

    # 5. Shoulder angle (s1)
    phi_line = math.atan2(z2, r2)
    phi_a    = _safe_acos((L1**2 + d**2 - L2**2) / (2 * L1 * d))
    s1       = clamp(math.degrees(phi_line + phi_a))

    # 6. Elbow angle (s2)
    angle_b = _safe_acos((L1**2 + L2**2 - d**2) / (2 * L1 * L2))
    s2      = clamp(math.degrees(angle_b) - 90.0)

    # 7. Wrist angle (s3) — keeps end-effector level
    s3 = clamp(90.0 - ((90.0 - s1) - s2))

    return tuple(int(round(clamp(a))) for a in (s0, s1, s2, s3))

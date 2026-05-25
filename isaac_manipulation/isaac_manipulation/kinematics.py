"""Doosan M0609 6-DOF arm kinematics — numerical FK + Jacobian + DLS-IK.

System-humble (Python 3.10) compatible: numpy only, no isaacsim/orocos
dependency.

DH parameters are best-effort (Doosan M0609 datasheet + community URDFs).
Cross-check against vehicle_v3 USD m0609 articulation in Phase 3b-3.

API:
    fk(theta_rad) -> 4x4 transform of link_6 in arm base frame
    jacobian(theta_rad) -> 6x6 geometric Jacobian (3 linear + 3 angular)
    dls_ik(target_pos, target_R, theta0, ...) -> theta solution (best-effort)
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np


@dataclass
class DHRow:
    """Standard (proximal) Denavit-Hartenberg row: a, alpha, d, theta_offset.

    Joint variable adds to theta_offset:
        theta_i = q_i + theta_offset
    """
    a: float
    alpha: float
    d: float
    theta_offset: float


# Doosan M0609 best-effort standard DH (meters / radians).
# Reach ~900 mm, payload 6 kg. Cross-validate against USD joint geometry.
M0609_DH: List[DHRow] = [
    DHRow(a=0.0,    alpha=-math.pi / 2, d=0.1525, theta_offset=0.0),
    DHRow(a=0.411,  alpha=0.0,          d=0.0,    theta_offset=-math.pi / 2),
    DHRow(a=0.0,    alpha=math.pi / 2,  d=0.0,    theta_offset=math.pi / 2),
    DHRow(a=0.0,    alpha=-math.pi / 2, d=0.368,  theta_offset=0.0),
    DHRow(a=0.0,    alpha=math.pi / 2,  d=0.0,    theta_offset=0.0),
    DHRow(a=0.0,    alpha=0.0,          d=0.121,  theta_offset=0.0),
]


def _dh_transform(row: DHRow, q: float) -> np.ndarray:
    """Standard DH transform A_i from frame i-1 to frame i."""
    theta = q + row.theta_offset
    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(row.alpha), math.sin(row.alpha)
    return np.array([
        [ct, -st * ca,  st * sa, row.a * ct],
        [st,  ct * ca, -ct * sa, row.a * st],
        [0.0,      sa,       ca,      row.d],
        [0.0,     0.0,      0.0,        1.0],
    ], dtype=np.float64)


def fk_per_joint(q: np.ndarray, dh: List[DHRow] = M0609_DH) -> List[np.ndarray]:
    """Cumulative transforms [T_0_0, T_0_1, ..., T_0_n]."""
    T = np.eye(4, dtype=np.float64)
    Ts = [T.copy()]
    for i, row in enumerate(dh):
        T = T @ _dh_transform(row, float(q[i]))
        Ts.append(T.copy())
    return Ts


def fk(q: np.ndarray, dh: List[DHRow] = M0609_DH) -> np.ndarray:
    """Forward kinematics: 4x4 transform of last frame in base."""
    return fk_per_joint(q, dh)[-1]


def jacobian(q: np.ndarray, dh: List[DHRow] = M0609_DH) -> np.ndarray:
    """Geometric Jacobian (6 x n). Rows 0..2 = linear, 3..5 = angular.

    Revolute-joint formula:
        J_v_i = z_{i-1} x (p_e - p_{i-1})
        J_w_i = z_{i-1}
    """
    Ts = fk_per_joint(q, dh)
    n = len(dh)
    p_e = Ts[-1][:3, 3]
    J = np.zeros((6, n), dtype=np.float64)
    for i in range(n):
        z_prev = Ts[i][:3, 2]
        p_prev = Ts[i][:3, 3]
        J[:3, i] = np.cross(z_prev, p_e - p_prev)
        J[3:, i] = z_prev
    return J


def _rot_error(R_des: np.ndarray, R_cur: np.ndarray) -> np.ndarray:
    """Orientation error (axis-angle, in current frame, expressed in base)."""
    R_err = R_des @ R_cur.T
    cos_th = np.clip((np.trace(R_err) - 1.0) * 0.5, -1.0, 1.0)
    theta = math.acos(cos_th)
    if abs(theta) < 1e-9:
        return np.zeros(3, dtype=np.float64)
    if abs(theta - math.pi) < 1e-6:
        # near-pi degenerate — approximate
        diag = np.maximum(np.diag(R_err) + 1.0, 0.0) * 0.5
        axis = np.sqrt(diag)
        return theta * axis
    inv_2s = 1.0 / (2.0 * math.sin(theta))
    axis = np.array([
        R_err[2, 1] - R_err[1, 2],
        R_err[0, 2] - R_err[2, 0],
        R_err[1, 0] - R_err[0, 1],
    ]) * inv_2s
    return theta * axis


def dls_ik(target_pos: np.ndarray,
           target_R: Optional[np.ndarray],
           q_init: np.ndarray,
           dh: List[DHRow] = M0609_DH,
           max_iters: int = 60,
           pos_tol: float = 5e-4,
           rot_tol: float = 1e-2,
           damping: float = 0.05,
           step_clip: float = 0.25,
           q_min: Optional[np.ndarray] = None,
           q_max: Optional[np.ndarray] = None,
           position_only: bool = False) -> Tuple[np.ndarray, bool, float]:
    """Damped Least-Squares inverse kinematics.

    Args:
        target_pos: (3,) desired end-effector position in base frame (m).
        target_R: (3,3) desired orientation or None for position-only.
        q_init: (6,) initial joint angles (rad).
        position_only: if True, ignore orientation residual.

    Returns:
        (q, converged, final_err_norm)
    """
    q = q_init.astype(np.float64).copy()
    n = len(dh)
    last_err = float("inf")

    for _ in range(max_iters):
        T = fk(q, dh)
        p_err = target_pos - T[:3, 3]
        if position_only or target_R is None:
            err = p_err
            J = jacobian(q, dh)[:3, :]
        else:
            r_err = _rot_error(target_R, T[:3, :3])
            err = np.concatenate([p_err, r_err])
            J = jacobian(q, dh)

        last_err = float(np.linalg.norm(err))
        pos_ok = float(np.linalg.norm(p_err)) < pos_tol
        rot_ok = position_only or float(np.linalg.norm(_rot_error(target_R, T[:3, :3]))) < rot_tol
        if pos_ok and rot_ok:
            return q, True, last_err

        # DLS: dq = J^T (J J^T + lambda^2 I)^-1 e
        m = J.shape[0]
        JJt = J @ J.T + (damping ** 2) * np.eye(m)
        try:
            dq = J.T @ np.linalg.solve(JJt, err)
        except np.linalg.LinAlgError:
            return q, False, last_err

        # Clip step magnitude for stability
        norm = float(np.linalg.norm(dq))
        if norm > step_clip:
            dq *= step_clip / norm
        q = q + dq

        if q_min is not None:
            q = np.maximum(q, q_min)
        if q_max is not None:
            q = np.minimum(q, q_max)

    return q, False, last_err


def deg(rad: np.ndarray) -> np.ndarray:
    return np.degrees(rad)


def rad(deg_arr) -> np.ndarray:
    return np.radians(np.asarray(deg_arr, dtype=np.float64))


# ─── quick sanity (run as module: python3 -m isaac_manipulation.kinematics) ──
def _selftest() -> None:
    home_deg = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]
    q0 = rad(home_deg)
    T_home = fk(q0)
    print(f"HOME ({home_deg} deg) → link_6 pos = {T_home[:3, 3].round(3)} m")

    # IK: pull TCP straight down 10 cm from HOME
    target_pos = T_home[:3, 3] + np.array([0.0, 0.0, -0.10])
    target_R = T_home[:3, :3]
    q_sol, ok, err = dls_ik(target_pos, target_R, q0)
    T_sol = fk(q_sol)
    print(f"IK target {target_pos.round(3)}  →  ok={ok}  err={err:.5f}")
    print(f"  joint sol (deg) = {deg(q_sol).round(2)}")
    print(f"  achieved pos    = {T_sol[:3, 3].round(3)}")


if __name__ == "__main__":
    _selftest()

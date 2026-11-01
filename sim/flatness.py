"""Differential flatness of a quadrotor with a cable-suspended point-mass load (Paper 1: Sreenath, Lee, Kumar,
CDC 2013, Section III). Flat outputs: payload position x_L and drone yaw psi.

Convention (Paper 1): q is the unit vector from the drone (attachment at its CoM) to the payload, l the cable
length, T the cable tension, e3 = (0, 0, 1).
    payload :  m_L (x_L'' + g e3) = -T q
    drone   :  m_Q (x_Q'' + g e3) = f R e3 + T q,       x_Q = x_L - l q
Map:
    u     = x_L'' + g e3,   q = -u/|u|,   T = m_L |u|
    F     = f R e3 = (m_Q + m_L) u - m_Q l q''          (needs x_L up to 4th derivative)
    R     = [b1 b2 b3] from b3 = F/|F| and yaw psi
    Omega = (R^T R')^vee  (5th),   Omega' (6th),   M = J Omega' + Omega x J Omega
All derivatives are exact via jet arithmetic (sim/jet.py).
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .jet import Jet, columns_to_matrix, vee

E3 = np.array([0.0, 0.0, 1.0])


class FlatnessSingularity(ValueError):
    """The flatness map is undefined (cable tension -> 0, or thrust direction parallel to the yaw heading)."""


@dataclass
class FlatParams:
    m_Q: float
    m_L: float
    l: float
    J: np.ndarray        # 3x3 drone inertia about its CoM, body axes
    g: float = 9.81


@dataclass
class FlatState:
    x_L: np.ndarray
    v_L: np.ndarray
    a_L: np.ndarray
    q: np.ndarray
    q_dot: np.ndarray
    q_ddot: np.ndarray
    omega: np.ndarray          # cable angular velocity q x q_dot
    omega_dot: np.ndarray      # q x q_ddot
    T: float                   # cable tension
    x_Q: np.ndarray
    v_Q: np.ndarray
    a_Q: np.ndarray
    f: float                   # collective thrust
    R: np.ndarray
    Omega: np.ndarray          # body angular velocity
    Omega_dot: np.ndarray
    M: np.ndarray              # body torque about the CoM
    q_jet: Jet = None          # full jets (for controllers that need further derivatives)
    R_jet: Jet = None


def flat_state(xL_derivs: np.ndarray, p: FlatParams, psi_derivs: np.ndarray | None = None,
               min_tension_acc: float = 1e-3) -> FlatState:
    """xL_derivs: (7, 3) payload position and derivatives 1..6; psi_derivs: (3,) yaw and its first 2 derivatives
    (default 0, Paper 1 Eq. 47)."""
    xL = np.asarray(xL_derivs, float)
    if xL.shape[0] < 7:
        raise ValueError("flatness needs payload derivatives up to 6th order")
    psi = np.zeros(3) if psi_derivs is None else np.asarray(psi_derivs, float)

    # cable direction and tension (u has derivatives 0..4 from x_L 2..6)
    u = Jet(xL[2:7].copy())
    u.d[0] = u.d[0] + p.g * E3
    if np.linalg.norm(u.val) < min_tension_acc:
        raise FlatnessSingularity("|x_L'' + g e3| ~ 0: cable tension vanishes (payload in free fall)")
    u_hat, u_norm = u.normalized()
    q = -u_hat                                   # derivatives 0..4
    T = p.m_L * u_norm.val

    # thrust vector F = (m_Q + m_L) u - m_Q l q''  (derivatives 0..2)
    F = (p.m_Q + p.m_L) * u.truncate(2) - p.m_Q * p.l * q.shift(2)
    b3, f = F.normalized()

    # attitude from b3 and yaw
    psi_j = Jet(psi)
    c, s = _cos_sin(psi_j)
    zero = Jet.constant(0.0, 2)
    b1c = Jet(np.stack([c.d, s.d, zero.d], axis=-1))
    b2_raw = b3.cross(b1c)
    if np.linalg.norm(b2_raw.val) < 1e-6:
        raise FlatnessSingularity("thrust direction parallel to the yaw heading")
    b2, _ = b2_raw.normalized()
    b1 = b2.cross(b3)
    R = columns_to_matrix(b1, b2, b3)            # derivatives 0..2

    Rt = R.T()
    Omega = vee(Rt.d[0] @ R.d[1])
    Omega_dot = vee(Rt.d[0] @ R.d[2])            # R^T R'' = Omega_hat' + Omega_hat^2 (symmetric part dropped)
    M = p.J @ Omega_dot + np.cross(Omega, p.J @ Omega)

    xQ = Jet(xL[:5]) - p.l * q                   # derivatives 0..4
    return FlatState(
        x_L=xL[0], v_L=xL[1], a_L=xL[2],
        q=q.d[0], q_dot=q.d[1], q_ddot=q.d[2],
        omega=np.cross(q.d[0], q.d[1]), omega_dot=np.cross(q.d[0], q.d[2]),
        T=float(T),
        x_Q=xQ.d[0], v_Q=xQ.d[1], a_Q=xQ.d[2],
        f=float(f.val), R=R.d[0], Omega=Omega, Omega_dot=Omega_dot, M=M,
        q_jet=q, R_jet=R,
    )


def _cos_sin(psi: Jet) -> tuple[Jet, Jet]:
    """cos and sin of a scalar jet (exact derivatives via exp(i psi) recursion)."""
    n = psi.order
    a = 1j * psi.d
    h = np.zeros(n + 1, complex)
    h[0] = np.exp(a[0])
    from math import comb
    for m in range(1, n + 1):
        h[m] = sum(comb(m - 1, k) * a[k + 1] * h[m - 1 - k] for k in range(m))
    return Jet(h.real), Jet(h.imag)


def wrench(fs: FlatState) -> np.ndarray:
    """CTBM command (f, M) for the actuator stack."""
    return np.array([fs.f, *fs.M])

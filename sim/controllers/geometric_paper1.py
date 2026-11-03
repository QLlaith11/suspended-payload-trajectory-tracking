"""Paper 1: geometric control of a quadrotor with a cable-suspended load (Sreenath, Lee, Kumar, CDC 2013, Sec. IV).

Three flight modes, each built on the previous one:
  (a) quadrotor attitude   Eq. (21)
  (b) load attitude        Eqs. (25)-(32)
  (c) load position        Eqs. (36)-(38)

Implementation decisions (SIMULATION_DESIGN.md / IMPLEMENTATION_PLAN.md step 2.2):
- Rates of the *computed* commands (Omega_c, Omega_c_dot from R_c; q_c_dot, q_c_ddot from q_c): option "flat" uses the
  exact rates of the reference from the flatness map (main version); "numdiff" differentiates the computed commands
  numerically (filtered); "zero" ignores them. "numdiff" and "zero" are ablations.
- Gains from bandwidths (time-scale separation): k_x = M w_x^2, k_v = 2 zeta M w_x; k_q = m_Q l w_q^2,
  k_w = 2 zeta m_Q l w_q; attitude K_R = w_R^2 J, K_Omega = 2 zeta w_R J (per-axis; the paper's 1/eps^2 k_R and
  1/eps k_Omega with a scalar k_R, scaled by the inertia so every axis has the same bandwidth).
  Defaults 1.5 / 7.5 / 30 rad/s (finding F3): Paper 1's time-scale separation needs ~5x between loops, and the
  X2's 13 N rotors cap the attitude bandwidth at ~30 rad/s (faster gains saturate and diverge on 30 deg swings).
- The cable-rate error is the tangent vector e_qdot = q_dot - (q_d x q_d_dot) x q  (Eq. 18).
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..flatness import FlatParams, FlatState

E3 = np.array([0.0, 0.0, 1.0])


def hat(v: np.ndarray) -> np.ndarray:
    return np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])


def vee(S: np.ndarray) -> np.ndarray:
    return np.array([S[2, 1], S[0, 2], S[1, 0]])


def so3_log_vee(R: np.ndarray) -> np.ndarray:
    """Rotation vector of R (axis * angle)."""
    c = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    angle = np.arccos(c)
    if angle < 1e-9:
        return vee(R - R.T) / 2.0
    return angle / (2.0 * np.sin(angle)) * vee(R - R.T)


@dataclass
class Gains:
    k_x: float
    k_v: float
    k_q: float
    k_w: float
    K_R: np.ndarray
    K_Omega: np.ndarray
    bandwidths: tuple

    @staticmethod
    def from_bandwidths(p: FlatParams, w_x: float = 1.5, w_q: float = 7.5, w_R: float = 30.0,
                        zeta: float = 0.9) -> "Gains":
        M, mQl = p.m_Q + p.m_L, p.m_Q * p.l
        J = np.asarray(p.J)
        return Gains(k_x=M * w_x**2, k_v=2 * zeta * M * w_x,
                     k_q=mQl * w_q**2, k_w=2 * zeta * mQl * w_q,
                     K_R=w_R**2 * J, K_Omega=2 * zeta * w_R * J, bandwidths=(w_x, w_q, w_R))


class _Differentiator:
    """Filtered numerical derivative of the computed commands (ablation option "numdiff")."""

    def __init__(self, dt: float, tau: float = 0.01):
        self.dt, self.alpha = dt, dt / (tau + dt)
        self.prev_R = self.prev_q = None
        self.Om = np.zeros(3)
        self.Om_dot = np.zeros(3)
        self.q_dot = np.zeros(3)
        self.q_ddot = np.zeros(3)

    def update_R(self, Rc: np.ndarray):
        if self.prev_R is not None:
            Om_raw = so3_log_vee(self.prev_R.T @ Rc) / self.dt
            Om_new = self.Om + self.alpha * (Om_raw - self.Om)
            self.Om_dot = self.Om_dot + self.alpha * ((Om_new - self.Om) / self.dt - self.Om_dot)
            self.Om = Om_new
        self.prev_R = Rc.copy()
        return self.Om, self.Om_dot

    def update_q(self, qc: np.ndarray):
        if self.prev_q is not None:
            qd_raw = (qc - self.prev_q) / self.dt
            qd_new = self.q_dot + self.alpha * (qd_raw - self.q_dot)
            self.q_ddot = self.q_ddot + self.alpha * ((qd_new - self.q_dot) / self.dt - self.q_ddot)
            self.q_dot = qd_new
        self.prev_q = qc.copy()
        return self.q_dot, self.q_ddot


class GeometricController:
    """Paper 1 controller. Call one of attitude(), load_attitude(), load_position() each control tick."""

    def __init__(self, p: FlatParams, gains: Gains, dt: float, derivative_source: str = "flat",
                 b1d: np.ndarray = np.array([1.0, 0.0, 0.0])):
        if derivative_source not in ("flat", "numdiff", "zero"):
            raise ValueError(derivative_source)
        self.p, self.k, self.dt = p, gains, dt
        self.source = derivative_source
        self.b1d = np.asarray(b1d, float)
        self.diff = _Differentiator(dt)
        self.last = {}

    # ------------------------------------------------------------------ (a) quadrotor attitude, Eq. (21)
    def attitude_moment(self, R, Omega, Rd, Omega_d, Omega_d_dot) -> np.ndarray:
        J, k = self.p.J, self.k
        e_R = 0.5 * vee(Rd.T @ R - R.T @ Rd)
        e_Om = Omega - R.T @ Rd @ Omega_d
        M = (-k.K_R @ e_R - k.K_Omega @ e_Om + np.cross(Omega, J @ Omega)
             - J @ (hat(Omega) @ R.T @ Rd @ Omega_d - R.T @ Rd @ Omega_d_dot))
        self.last.update(e_R=e_R, e_Omega=e_Om, Psi_R=0.5 * np.trace(np.eye(3) - Rd.T @ R))
        return M

    def attitude(self, s, f: float, Rd, Omega_d=np.zeros(3), Omega_d_dot=np.zeros(3)) -> np.ndarray:
        return np.array([f, *self.attitude_moment(s.R, s.Omega, Rd, Omega_d, Omega_d_dot)])

    # ------------------------------------------------------------------ (b) load attitude, Eqs. (25)-(32)
    @staticmethod
    def cable_rate_error(q, q_dot, qd, qd_dot) -> np.ndarray:
        """Eq. (18): e_qdot = q_dot - (q_d x q_d_dot) x q, a tangent vector at q."""
        return q_dot - np.cross(np.cross(qd, qd_dot), q)

    def thrust_vector(self, s, qd, qd_dot, qd_ddot, F_n) -> np.ndarray:
        k, mQl = self.k, self.p.m_Q * self.p.l
        q, q_dot = s.q, s.q_dot
        e_q = hat(q) @ hat(q) @ qd                                    # Eq. (17)
        e_qdot = self.cable_rate_error(q, q_dot, qd, qd_dot)          # Eq. (18)
        F_pd = -k.k_q * e_q - k.k_w * e_qdot                          # Eq. (29)
        F_ff = (mQl * (q @ np.cross(qd, qd_dot)) * np.cross(q, q_dot)
                + mQl * np.cross(np.cross(qd, qd_ddot), q))           # Eq. (30)
        self.last.update(e_q=e_q, e_qdot=e_qdot, Psi_q=1.0 - qd @ q)
        return F_n - F_pd - F_ff                                       # Eq. (27)

    def computed_attitude(self, F: np.ndarray) -> np.ndarray:
        b3c = F / np.linalg.norm(F)                                   # Eq. (26)
        c = np.cross(b3c, self.b1d)
        b1c = -np.cross(b3c, c) / np.linalg.norm(c)                   # Eq. (31)
        return np.column_stack([b1c, np.cross(b3c, b1c), b3c])       # Eq. (25)

    def _attitude_rates(self, Rc, flat: FlatState | None):
        if self.source == "flat" and flat is not None:
            return flat.Omega, flat.Omega_dot
        if self.source == "numdiff":
            return self.diff.update_R(Rc)
        return np.zeros(3), np.zeros(3)

    def load_attitude(self, s, qd, qd_dot=np.zeros(3), qd_ddot=np.zeros(3), F_n=None,
                      flat: FlatState | None = None) -> np.ndarray:
        """Mode (b). Default F_n = -(q_d . q) q (Eq. 28)."""
        if F_n is None:
            F_n = -(qd @ s.q) * s.q
        F = self.thrust_vector(s, qd, qd_dot, qd_ddot, F_n)
        Rc = self.computed_attitude(F)
        Om_c, Om_c_dot = self._attitude_rates(Rc, flat)
        f = F @ s.R[:, 2]                                             # Eq. (32)
        self.last.update(F=F, R_c=Rc, Omega_c=Om_c, Omega_c_dot=Om_c_dot, f=f)
        return np.array([f, *self.attitude_moment(s.R, s.Omega, Rc, Om_c, Om_c_dot)])

    # ------------------------------------------------------------------ (c) load position, Eqs. (36)-(38)
    def load_position(self, s, ref: np.ndarray, flat: FlatState | None = None) -> np.ndarray:
        """Mode (c). ref = (7, 3) payload reference derivatives; flat = flat state of the reference (feed-forward)."""
        p, k = self.p, self.k
        e_x, e_v = s.x_L - ref[0], s.v_L - ref[1]
        A = (-k.k_x * e_x - k.k_v * e_v + (p.m_Q + p.m_L) * (ref[2] + p.g * E3)
             + p.m_Q * p.l * (s.q_dot @ s.q_dot) * s.q)              # Eq. (37)
        qc = -A / np.linalg.norm(A)                                   # Eq. (36)
        if self.source == "flat" and flat is not None:
            qc_dot, qc_ddot = flat.q_dot, flat.q_ddot
        elif self.source == "numdiff":
            qc_dot, qc_ddot = self.diff.update_q(qc)
        else:
            qc_dot, qc_ddot = np.zeros(3), np.zeros(3)
        F_n = (A @ s.q) * s.q                                         # Eq. (38)
        self.last.update(e_x=e_x, e_v=e_v, A=A, q_c=qc)
        return self.load_attitude(s, qc, qc_dot, qc_ddot, F_n=F_n, flat=flat)

"""Body-rate controller for the CTBR interface (D8): the outer controller sends collective thrust + body-rate
command; this loop (500 Hz, D5) produces body torques.

    tau = J (w_rate (Omega_cmd - Omega) + k_i int(Omega_cmd - Omega)) + Omega x J Omega

Paper 1's attitude law (Eq. 21) with Omega_d = 0 is exactly a P attitude loop inside this P rate loop:
    Omega_cmd = R^T R_c Omega_c - k_att e_R,   k_att = w_R / (2 zeta),  w_rate = 2 zeta w_R
gives tau = -J w_rate k_att e_R - J w_rate e_Omega + ... = -K_R e_R - K_Omega e_Omega with K_R = w_R^2 J,
K_Omega = 2 zeta w_R J (Gains.from_bandwidths). The feed-forward torque of Eq. 21 (Omega_c_dot) is not part of the
CTBR interface and is dropped (as on PX4). Integral term: PX4-style, default 0 (equivalence check, step 2.6c-1).
"""
from __future__ import annotations

import numpy as np


class RateController:
    def __init__(self, J: np.ndarray, w_rate: float, k_i: float = 0.0, dt: float = 0.002):
        self.J, self.w, self.k_i, self.dt = np.asarray(J), w_rate, k_i, dt
        self.integ = np.zeros(3)

    def __call__(self, Omega: np.ndarray, Omega_cmd: np.ndarray) -> np.ndarray:
        e = Omega_cmd - Omega
        self.integ += e * self.dt
        return self.J @ (self.w * e + self.k_i * self.integ) + np.cross(Omega, self.J @ Omega)


def ctbr_from_paper1(ctrl, s, zeta: float = 0.9) -> tuple[float, np.ndarray]:
    """Convert Paper 1's last computed attitude command (ctrl.last after load_position/load_attitude) into a CTBR
    command (f, Omega_cmd), using the attitude bandwidth of ctrl's gains."""
    w_R = ctrl.k.bandwidths[2]
    k_att = w_R / (2 * zeta)
    Rc, Om_c = ctrl.last["R_c"], ctrl.last["Omega_c"]
    e_R = 0.5 * _vee(Rc.T @ s.R - s.R.T @ Rc)
    return float(ctrl.last["f"]), s.R.T @ Rc @ Om_c - k_att * e_R


def rate_bandwidth(w_R: float, zeta: float = 0.9) -> float:
    return 2 * zeta * w_R


def _vee(S):
    return np.array([S[2, 1], S[0, 2], S[1, 0]])

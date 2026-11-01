"""Actuator stack (D8): CTBM command -> mixer -> prioritised saturation -> rotor thrust commands.

CTBM = collective thrust f [N] + body torques (tau_x, tau_y, tau_z) [N m] about the drone CoM, body axes.
Motor lag is not here: it is a MuJoCo actuator filter configured in sim/model.py (D8, step 1.2).
The body-rate controller (CTBR interface) is added in step 2.6.
"""
from __future__ import annotations

import numpy as np

from .model import DroneInfo


def allocation_matrix(info: DroneInfo) -> np.ndarray:
    """B such that [f, tau_x, tau_y, tau_z] = B @ rotor_thrusts.

    Thrust f_i acts along body z at r_i (relative to the CoM): torque r_i x (0, 0, f_i) = (y_i f_i, -x_i f_i, 0).
    Reaction (yaw) torque is c_i f_i with the signed coefficient from the model's actuator gear.
    The rotors' height relative to the CoM does not create torque because thrust is parallel to body z.
    """
    r = info.rotor_pos - info.com
    return np.vstack([np.ones(4), r[:, 1], -r[:, 0], info.yaw_coeff])


class Mixer:
    """Wrench -> rotor thrusts with PX4-style priorities (sequential desaturation):
    1. roll/pitch torque (scaled down only if it alone is infeasible),
    2. collective thrust (shifted into the feasible range),
    3. yaw torque (reduced first).
    """

    def __init__(self, info: DroneInfo, f_min: float | None = None, f_max: float | None = None):
        self.B = allocation_matrix(info)
        self.B_inv = np.linalg.inv(self.B)
        self.f_min = info.thrust_min if f_min is None else f_min
        self.f_max = info.thrust_max if f_max is None else f_max
        self.w_thrust = self.B_inv[:, 0]      # rotor pattern per newton of collective thrust
        self.w_rp = self.B_inv[:, 1:3]        # roll/pitch patterns (sum to zero)
        self.w_yaw = self.B_inv[:, 3]         # yaw pattern (sums to zero)

    def unconstrained(self, wrench: np.ndarray) -> np.ndarray:
        return self.B_inv @ np.asarray(wrench, float)

    def wrench(self, thrusts: np.ndarray) -> np.ndarray:
        return self.B @ np.asarray(thrusts, float)

    def _thrust_interval(self, base: np.ndarray) -> tuple[float, float]:
        """Range of collective thrust T such that base + T*w_thrust stays within [f_min, f_max]."""
        lo = (self.f_min - base) / self.w_thrust
        hi = (self.f_max - base) / self.w_thrust
        return float(np.max(lo)), float(np.min(hi))

    def _feasible(self, thrusts: np.ndarray, tol: float = 1e-12) -> bool:
        return bool(np.all(thrusts >= self.f_min - tol) and np.all(thrusts <= self.f_max + tol))

    def _thrust_yaw_region(self, rp: np.ndarray, tz: float):
        """Constraints a*T + b*beta <= c (rotor bounds and 0 <= beta <= 1) for rotors = rp + T*w + beta*tz*w_yaw."""
        y = tz * self.w_yaw
        A = np.concatenate([self.w_thrust, -self.w_thrust, [0.0, 0.0]])
        Bc = np.concatenate([y, -y, [1.0, -1.0]])
        C = np.concatenate([self.f_max - rp, rp - self.f_min, [1.0, 0.0]])
        return A, Bc, C

    def _thrust_range_any_yaw(self, rp: np.ndarray, tz: float) -> tuple[float, float]:
        """Range of collective thrust T for which SOME yaw fraction beta in [0, 1] is feasible
        (projection of the 2-D feasible polygon onto T, from its vertices)."""
        A, Bc, C = self._thrust_yaw_region(rp, tz)
        ts = []
        for i in range(len(A)):
            for j in range(i + 1, len(A)):
                det = A[i] * Bc[j] - A[j] * Bc[i]
                if abs(det) < 1e-14:
                    continue
                T = (C[i] * Bc[j] - C[j] * Bc[i]) / det
                beta = (A[i] * C[j] - A[j] * C[i]) / det
                if np.all(A * T + Bc * beta <= C + 1e-9):
                    ts.append(T)
        return (min(ts), max(ts)) if ts else (np.nan, np.nan)

    def _max_yaw_fraction(self, rp: np.ndarray, T: float, tz: float) -> float:
        A, Bc, C = self._thrust_yaw_region(rp, tz)
        hi, lo = 1.0, 0.0
        for a, b, c in zip(A, Bc, C):
            if b > 1e-15:
                hi = min(hi, (c - a * T) / b)
            elif b < -1e-15:
                lo = max(lo, (c - a * T) / b)
        return max(lo, min(hi, 1.0))

    def __call__(self, wrench: np.ndarray) -> np.ndarray:
        f, tx, ty, tz = np.asarray(wrench, float)
        full = self.unconstrained(wrench)
        if self._feasible(full):                       # feasible command: returned exactly
            return full
        rp = self.w_rp @ np.array([tx, ty])

        # 1. roll/pitch: largest scale alpha in [0, 1] for which some collective thrust is feasible
        alpha = 1.0
        lo, hi = self._thrust_interval(rp)
        if lo > hi:
            a_lo, a_hi = 0.0, 1.0
            for _ in range(50):                         # bisection; feasibility is monotone in alpha
                a = 0.5 * (a_lo + a_hi)
                l, h = self._thrust_interval(a * rp)
                a_lo, a_hi = (a, a_hi) if l <= h else (a_lo, a)
            alpha = a_lo
            rp = alpha * rp
            lo, hi = self._thrust_interval(rp)

        # 2. collective thrust: closest value to the request for which some yaw (possibly zero) is feasible
        t_lo, t_hi = self._thrust_range_any_yaw(rp, tz)
        if np.isnan(t_lo):                              # numerical edge case: fall back to the yaw-free range
            t_lo, t_hi = lo, hi
        T = float(np.clip(f, t_lo, t_hi))

        # 3. yaw: largest fraction beta in [0, 1] of the requested yaw torque at that thrust
        beta = self._max_yaw_fraction(rp, T, tz)
        out = rp + T * self.w_thrust + beta * tz * self.w_yaw
        return np.clip(out, self.f_min, self.f_max)    # only removes round-off

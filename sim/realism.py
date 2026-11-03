"""L1 realism (D10): sensor noise on the state the controller sees, and action delay. Air drag is a model setting
(sim/model.py, realism.fluid_drag). The simulator always integrates the TRUE state; only the controller's view and
the timing of its commands are affected."""
from __future__ import annotations

from collections import deque
from dataclasses import replace

import numpy as np

from .state import State


def _exp_so3(w: np.ndarray) -> np.ndarray:
    a = np.linalg.norm(w)
    if a < 1e-12:
        return np.eye(3)
    k = w / a
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(a) * K + (1 - np.cos(a)) * K @ K


class SensorModel:
    """Mocap-like measurement (D10 L1): independent zero-mean Gaussian noise per axis on drone position, velocity,
    attitude (small random rotation, body frame), gyro, and payload position and velocity. The cable direction q and
    its rate are re-derived from the noisy positions/velocities (as a mocap-based estimator would)."""

    def __init__(self, cfg: dict, seed: int):
        r = cfg["realism"]
        self.on = bool(r["sensor_noise"])
        n = r["noise"]
        self.s_pos, self.s_vel, self.s_att, self.s_gyro = n["pos"], n["vel"], np.radians(n["att_deg"]), n["gyro"]
        self.rng = np.random.default_rng(seed)

    def __call__(self, s: State) -> State:
        if not self.on:
            return s
        g = self.rng.standard_normal
        R = s.R @ _exp_so3(self.s_att * g(3))
        m = replace(s, x_Q=s.x_Q + self.s_pos * g(3), v_Q=s.v_Q + self.s_vel * g(3), R=R,
                    Omega=s.Omega + self.s_gyro * g(3))
        if s.x_L is not None:
            x_L, v_L = s.x_L + self.s_pos * g(3), s.v_L + self.s_vel * g(3)
            d = x_L - m.x_Q
            length = np.linalg.norm(d)
            q = d / length
            q_dot = (v_L - m.v_Q) / length
            q_dot = q_dot - q * (q @ q_dot)
            m = replace(m, x_L=x_L, v_L=v_L, q=q, q_dot=q_dot, omega=np.cross(q, q_dot))
        return m


class CommandDelay:
    """Fixed transport delay of n controller ticks (FIFO). n = 0 passes commands through."""

    def __init__(self, delay_s: float, tick_s: float, initial):
        self.n = int(round(delay_s / tick_s))
        if abs(self.n * tick_s - delay_s) > 1e-9:
            raise ValueError(f"delay {delay_s} s is not a multiple of the controller period {tick_s} s")
        self.buf = deque([initial] * self.n)

    def __call__(self, cmd):
        if self.n == 0:
            return cmd
        self.buf.append(cmd)
        return self.buf.popleft()

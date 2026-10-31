"""State extraction in the notation of Sreenath, Lee, Kumar (CDC 2013), Paper 1.

x_Q, v_Q : drone CoM position / velocity (world)       R, Omega : drone attitude, body angular velocity
x_L, v_L : payload position / velocity (world)         q        : unit cable vector, drone attachment -> payload
q_dot    : rate of q                                   omega    : cable angular velocity, q x q_dot
T        : cable tension (signed; > 0 = pulling),  from  m_L (a_L + g e3) = -T q   (Paper 1, Eq. 6/7)
"""
from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np

from .model import ATTACH_SITE, DRONE_BODY, PAYLOAD_BODY, PAYLOAD_SITE


@dataclass
class State:
    t: float
    x_Q: np.ndarray
    v_Q: np.ndarray
    R: np.ndarray
    Omega: np.ndarray
    x_L: np.ndarray | None = None
    v_L: np.ndarray | None = None
    q: np.ndarray | None = None
    q_dot: np.ndarray | None = None
    omega: np.ndarray | None = None
    cable_length: float | None = None


def _site_velocity(model, data, site: str) -> np.ndarray:
    v = np.zeros(6)  # [angular; linear], world frame, at the site position
    mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_SITE, model.site(site).id, v, 0)
    return v[3:]


def get_state(model: mujoco.MjModel, data: mujoco.MjData) -> State:
    b = model.body(DRONE_BODY).id
    R = data.xmat[b].reshape(3, 3).copy()
    Omega = data.qvel[3:6].copy()                    # free joint: angular velocity in body axes
    r_com = data.xipos[b] - data.xpos[b]             # body origin -> CoM, world axes
    v_Q = data.qvel[0:3] + np.cross(R @ Omega, r_com)  # free joint: linear velocity of the body origin
    s = State(t=float(data.time), x_Q=data.xipos[b].copy(), v_Q=v_Q, R=R, Omega=Omega)
    if PAYLOAD_SITE in [model.site(i).name for i in range(model.nsite)]:
        x_A = data.site_xpos[model.site(ATTACH_SITE).id]
        x_L = data.site_xpos[model.site(PAYLOAD_SITE).id].copy()
        v_A = _site_velocity(model, data, ATTACH_SITE)
        v_L = _site_velocity(model, data, PAYLOAD_SITE)
        d = x_L - x_A
        length = float(np.linalg.norm(d))
        q = d / length
        q_dot = (v_L - v_A) / length
        q_dot -= q * (q @ q_dot)                     # exact for a rigid link; removes round-off along q
        s.x_L, s.v_L, s.q, s.q_dot, s.omega, s.cable_length = x_L, v_L, q, q_dot, np.cross(q, q_dot), length
    return s


def cable_tension(model: mujoco.MjModel, data: mujoco.MjData, g: float) -> float:
    """Signed cable tension at the current state: T = -m_L q . (a_L + g e3).

    Recomputes accelerations at the current state (mj_forward does not change the state).
    """
    mujoco.mj_forward(model, data)
    mujoco.mj_rnePostConstraint(model, data)
    acc = np.zeros(6)  # [angular; linear]; MuJoCo's linear part is the proper acceleration a - g_vec = a + g e3
    mujoco.mj_objectAcceleration(model, data, mujoco.mjtObj.mjOBJ_SITE, model.site(PAYLOAD_SITE).id, acc, 0)
    m_L = float(model.body_mass[model.body(PAYLOAD_BODY).id])
    q = get_state(model, data).q
    return float(-m_L * q @ acc[3:])

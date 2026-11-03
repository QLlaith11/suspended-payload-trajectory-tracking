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
    cable_length_rate: float | None = None     # d|x_L - x_A|/dt (0 for C1; nonzero for C2 while slack)


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
        length_rate = float(q @ (v_L - v_A))
        q_dot -= q * (q @ q_dot)                     # tangent part (exact for C1; for C2 the radial part = length
                                                     # rate, reported separately in cable_length_rate)
        s.x_L, s.v_L, s.q, s.q_dot, s.omega, s.cable_length = x_L, v_L, q, q_dot, np.cross(q, q_dot), length
        s.cable_length_rate = length_rate
    return s


def set_state(model: mujoco.MjModel, data: mujoco.MjData, x_Q, v_Q, R, Omega, q=None, q_dot=None) -> None:
    """Put the simulator exactly into a given state (drone CoM position/velocity, attitude, body rates and, for
    cable C1, cable direction and rate). Inverse of get_state; used to start from a flat state."""
    b = model.body(DRONE_BODY).id
    R = np.asarray(R, float)
    com = model.body_ipos[b]                                   # CoM in the body frame
    quat = np.zeros(4)
    mujoco.mju_mat2Quat(quat, R.reshape(9))
    data.qpos[0:3] = np.asarray(x_Q) - R @ com                 # free joint position = body origin
    data.qpos[3:7] = quat
    w_drone = R @ np.asarray(Omega, float)                     # drone angular velocity, world axes
    data.qvel[0:3] = np.asarray(v_Q) - np.cross(w_drone, R @ com)
    data.qvel[3:6] = Omega
    if q is not None and model.njnt > 1 and model.jnt_type[1] == mujoco.mjtJoint.mjJNT_FREE:   # C2: free payload
        q, q_dot = np.asarray(q, float), np.asarray(q_dot, float)
        L = float(model.tendon_range[0, 1])
        a_loc = model.site_pos[model.site(ATTACH_SITE).id]
        x_A = data.qpos[0:3] + R @ a_loc
        v_A = data.qvel[0:3] + np.cross(w_drone, R @ a_loc)
        data.qpos[7:10] = x_A + L * q
        data.qpos[10:14] = [1.0, 0.0, 0.0, 0.0]
        data.qvel[6:9] = v_A + L * q_dot
        data.qvel[9:12] = 0.0
        mujoco.mj_forward(model, data)
        return
    if q is not None:
        if model.nq < 11:
            raise ValueError("model has no C1 cable joint")
        q, q_dot = np.asarray(q, float), np.asarray(q_dot, float)
        v = R.T @ q                                            # cable direction in drone axes
        down = np.array([0.0, 0.0, -1.0])
        axis = np.cross(down, v)
        s, c = np.linalg.norm(axis), float(down @ v)
        angle = np.arctan2(s, c)
        axis = axis / s if s > 1e-12 else np.array([1.0, 0.0, 0.0])
        q_ball = np.array([np.cos(angle / 2), *(np.sin(angle / 2) * axis)])
        data.qpos[7:11] = q_ball
        R_ball = np.zeros(9)
        mujoco.mju_quat2Mat(R_ball, q_ball)
        R_child = R @ R_ball.reshape(3, 3)
        # child angular velocity: swing q x q_dot, twist taken equal to the drone's (no relative twist)
        w_child = np.cross(q, q_dot) + (w_drone @ q) * q
        data.qvel[6:9] = R_child.T @ (w_child - w_drone)       # ball joint: relative, child axes
    mujoco.mj_forward(model, data)


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

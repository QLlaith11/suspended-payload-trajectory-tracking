"""Step 1.3 validation checks for cable C1 (rigid massless link) + payload P1 (point mass): gates P1-P7."""
from __future__ import annotations

import mujoco
import numpy as np

from .actuators import Mixer
from .config import deep_merge
from .model import build_model, drone_info
from .state import cable_tension, get_state

BALL_QPOS = slice(7, 11)   # free joint (7) then ball joint quaternion (4)
BALL_QVEL = slice(6, 9)    # free joint (6) then ball joint angular velocity (3)


def _tilt_payload(data, angle_rad: float, axis=(0.0, 1.0, 0.0)) -> None:
    axis = np.asarray(axis, float) / np.linalg.norm(axis)
    data.qpos[BALL_QPOS] = [np.cos(angle_rad / 2), *(np.sin(angle_rad / 2) * axis)]


def _masses(model, cfg):
    m_Q = drone_info(model).mass
    return m_Q, cfg["payload"]["mass"], m_Q + cfg["payload"]["mass"]


def p1_static_hover(cfg: dict, duration: float = 2.0) -> dict:
    model, data = build_model(cfg)
    g = cfg["sim"]["gravity"]
    m_L = cfg["payload"]["mass"]
    s0 = get_state(model, data)
    drift = swing = t_err = 0.0
    for _ in range(int(round(duration / model.opt.timestep))):
        mujoco.mj_step(model, data)
        s = get_state(model, data)
        drift = max(drift, float(np.linalg.norm(s.x_Q - s0.x_Q)))
        swing = max(swing, float(np.degrees(np.arccos(np.clip(-s.q[2], -1, 1)))))
    t_err = abs(cable_tension(model, data, g) - m_L * g)
    return {"max_drift_m": drift, "max_swing_deg": swing, "tension_err_N": t_err,
            "passed": drift < 1e-9 and swing < 1e-6 and t_err < 1e-9}


def _period(cfg: dict, length: float, amp_deg: float = 2.0, duration: float = 12.0):
    c = deep_merge(cfg, {"cable": {"length": length}})
    model, data = build_model(c)
    _tilt_payload(data, np.radians(amp_deg))
    mujoco.mj_forward(model, data)
    t, qx = [], []
    for _ in range(int(round(duration / model.opt.timestep))):
        mujoco.mj_step(model, data)
        t.append(data.time)
        qx.append(get_state(model, data).q[0])
    t, qx = np.array(t), np.array(qx)
    idx = np.where((qx[:-1] < 0) & (qx[1:] >= 0))[0]                 # upward zero crossings
    crossings = t[idx] - qx[idx] * (t[idx + 1] - t[idx]) / (qx[idx + 1] - qx[idx])
    m_Q, m_L, M = _masses(model, c)
    period_theory = 2 * np.pi / np.sqrt(c["sim"]["gravity"] * M / (m_Q * length))
    return float(np.mean(np.diff(crossings))), float(period_theory), (t, qx)


def p2_swing_period(cfg: dict) -> dict:
    out, series = {}, {}
    ok = True
    for L in (0.25, 0.5, 1.0):
        measured, theory, (t, qx) = _period(cfg, L)
        rel = abs(measured - theory) / theory
        out[f"L{L}_period_s"] = measured
        out[f"L{L}_theory_s"] = theory
        out[f"L{L}_rel_err"] = rel
        ok &= rel < 1e-3
        series[f"L{L}"] = (t, qx)
    return {**out, "passed": ok, "series": series}


def p3_energy(cfg: dict, duration: float = 2.0) -> dict:
    """Motors off (free fall), payload swinging 30 deg and drone tumbling: total mechanical energy conserved."""
    model, data = build_model(cfg)
    _tilt_payload(data, np.radians(30))
    data.qvel[3:6] = [0.5, -0.3, 0.2]
    data.qvel[BALL_QVEL] = [1.0, 2.0, 0.0]
    data.ctrl[:] = 0.0
    mujoco.mj_forward(model, data)
    E, length_err = [], 0.0
    L = cfg["cable"]["length"]
    for _ in range(int(round(duration / model.opt.timestep))):
        data.ctrl[:] = 0.0
        mujoco.mj_step(model, data)
        mujoco.mj_energyPos(model, data)
        mujoco.mj_energyVel(model, data)
        E.append(data.energy[0] + data.energy[1])
        length_err = max(length_err, abs(get_state(model, data).cable_length - L))
    E = np.array(E)
    return {"energy_drift_J": float(np.ptp(E)), "max_length_err_m": float(length_err),
            "passed": np.ptp(E) < 1e-6 and length_err < 1e-12}


def p4_length(cfg: dict, duration: float = 3.0) -> dict:
    """Cable length stays exactly L during an aggressive open-loop manoeuvre with thrust and torques."""
    model, data = build_model(cfg)
    mixer = Mixer(drone_info(model))
    m_Q, m_L, M = _masses(model, cfg)
    L, err = cfg["cable"]["length"], 0.0
    while data.time < duration:
        tau = [0.3, -0.2, 0.05] if 0.5 <= data.time < 0.8 else [0.0, 0.0, 0.0]
        data.ctrl[:] = mixer([1.3 * M * 9.81, *tau])
        mujoco.mj_step(model, data)
        err = max(err, abs(get_state(model, data).cable_length - L))
    return {"max_length_err_m": float(err), "passed": err < 1e-12}


def _manoeuvre(cfg: dict, dt: float, duration: float = 1.5):
    model, data = build_model(deep_merge(cfg, {"sim": {"dt": dt}}))
    mixer = Mixer(drone_info(model))
    M = _masses(model, cfg)[2]
    while data.time < duration - 1e-12:
        tau = [0.15, -0.1, 0.03] if 0.2 <= data.time < 0.5 else [0.0, 0.0, 0.0]
        data.ctrl[:] = mixer([M * 9.81, *tau])
        mujoco.mj_step(model, data)
    s = get_state(model, data)
    return s.x_Q, s.x_L


def p5_timestep(cfg: dict) -> dict:
    (q1, l1), (q2, l2) = _manoeuvre(cfg, 0.001), _manoeuvre(cfg, 0.0005)
    dq, dl = float(np.linalg.norm(q1 - q2)), float(np.linalg.norm(l1 - l2))
    return {"drone_pos_diff_m": dq, "payload_pos_diff_m": dl, "passed": dq < 1e-6 and dl < 1e-6}


def p6_state_consistency(cfg: dict) -> dict:
    """Velocities, q_dot and tension from sim/state.py vs central finite differences of the simulated motion.

    The finite-difference reference itself has an O(dt^2) error, so a fixed threshold is not meaningful.
    Convergence test instead: halving dt must reduce every error by ~4x (ratio in [3.5, 4.5]), which shows the
    remaining error is the reference's truncation error and the extracted quantities are exact in the limit.
    """
    e1 = _p6_errors(cfg, 0.001)
    e2 = _p6_errors(cfg, 0.0005)
    out, ok = {}, True
    for k in e1:
        ratio = e1[k] / e2[k]
        out[f"{k}_dt1ms"] = e1[k]
        out[f"{k}_ratio"] = ratio
        ok &= 3.5 <= ratio <= 4.5 and e1[k] < 1e-4
    return {**out, "passed": ok}


def _p6_errors(cfg: dict, dt_sim: float, duration: float = 1.5) -> dict:
    cfg = deep_merge(cfg, {"sim": {"dt": dt_sim}})
    model, data = build_model(cfg)
    mixer = Mixer(drone_info(model))
    M = _masses(model, cfg)[2]
    m_L, g = cfg["payload"]["mass"], cfg["sim"]["gravity"]
    _tilt_payload(data, np.radians(20), axis=(1.0, 1.0, 0.0))
    mujoco.mj_forward(model, data)
    dt = model.opt.timestep
    rec = {k: [] for k in ("x_Q", "v_Q", "q", "q_dot", "x_L", "v_L", "T")}
    while data.time < duration:
        tau = [0.1, -0.05, 0.02] if 0.3 <= data.time < 0.6 else [0.0, 0.0, 0.0]
        data.ctrl[:] = mixer([M * g, *tau])
        mujoco.mj_step(model, data)
        s = get_state(model, data)
        for k in ("x_Q", "v_Q", "q", "q_dot", "x_L", "v_L"):
            rec[k].append(getattr(s, k))
        rec["T"].append(cable_tension(model, data, g))
    r = {k: np.array(v) for k, v in rec.items()}

    def fd(x):  # central difference, interior points
        return (x[2:] - x[:-2]) / (2 * dt)

    keep = slice(1, -1)
    # avoid the two control discontinuities (torque pulse on/off) for the acceleration-based check
    errs = {
        "v_Q_err": float(np.max(np.abs(fd(r["x_Q"]) - r["v_Q"][keep]))),
        "q_dot_err": float(np.max(np.abs(fd(r["q"]) - r["q_dot"][keep]))),
        "v_L_err": float(np.max(np.abs(fd(r["x_L"]) - r["v_L"][keep]))),
    }
    a_L = fd(r["v_L"])
    T_fd = m_L * np.linalg.norm(a_L + np.array([0, 0, g]), axis=1)
    t = np.arange(1, len(r["T"]) - 1) * dt + dt
    smooth = (np.abs(t - 0.3) > 0.005) & (np.abs(t - 0.6) > 0.005)
    errs["tension_err_N"] = float(np.max(np.abs(T_fd[smooth] - r["T"][keep][smooth])))
    return errs


def p7_offset_attachment(cfg: dict) -> dict:
    """Attachment 7.7 cm below the CoM (Paper 3 realistic case): hover equilibrium still holds; a displaced payload
    now creates a torque on the drone, whereas attachment at the CoM creates none."""
    out = {}
    for label, off in (("at_com", 0.0), ("offset", -0.077)):
        c = deep_merge(cfg, {"cable": {"attach_offset_z": off}})
        model, data = build_model(c)
        x0 = get_state(model, data).x_Q
        for _ in range(2000):
            mujoco.mj_step(model, data)
        out[f"{label}_hover_drift_m"] = float(np.linalg.norm(get_state(model, data).x_Q - x0))
        model, data = build_model(c)
        _tilt_payload(data, np.radians(10))
        mujoco.mj_forward(model, data)
        out[f"{label}_drone_ang_acc"] = float(np.linalg.norm(data.qacc[3:6]))
    passed = (out["at_com_hover_drift_m"] < 1e-9 and out["offset_hover_drift_m"] < 1e-9
              and out["at_com_drone_ang_acc"] < 1e-9 and out["offset_drone_ang_acc"] > 1e-2)
    return {**out, "passed": passed}


ALL_CHECKS = {"P1_static_hover": p1_static_hover, "P2_swing_period": p2_swing_period, "P3_energy": p3_energy,
              "P4_length": p4_length, "P5_timestep": p5_timestep, "P6_state_consistency": p6_state_consistency,
              "P7_offset_attachment": p7_offset_attachment}

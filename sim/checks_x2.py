"""Step 1.1 validation checks for the bare Skydio X2 model (gates G1-G5, IMPLEMENTATION_PLAN.md).

Each check returns a dict with its measured values, the threshold and a `passed` flag, plus time series
for plotting. Used by both the unit tests (tests/test_model.py) and the experiment (e11_x2_validation).
"""
from __future__ import annotations

import mujoco
import numpy as np

from .config import deep_merge
from .model import build_model, drone_info

# Values measured in the design review (SIMULATION_DESIGN.md, D2)
EXPECTED_MASS = 1.325
EXPECTED_INERTIA = np.array([[0.03665, 0.0, -0.0021], [0.0, 0.02541, 0.0], [-0.0021, 0.0, 0.06053]])
EXPECTED_COM = np.array([0.0, 0.0, 0.054])


def _quat_angle_deg(q: np.ndarray) -> float:
    """Rotation angle of a unit quaternion relative to identity [deg]."""
    return float(np.degrees(2 * np.arccos(np.clip(abs(q[0]), -1.0, 1.0))))


def g1_properties(cfg: dict) -> dict:
    model, _ = build_model(cfg)
    info = drone_info(model)
    mass_err = abs(info.mass - EXPECTED_MASS)
    inertia_err = float(np.max(np.abs(info.inertia - EXPECTED_INERTIA)))
    com_err = float(np.max(np.abs(info.com - EXPECTED_COM)))
    return {
        "mass_kg": info.mass, "mass_err": mass_err,
        "inertia_max_err": inertia_err, "com_max_err_m": com_err,
        "passed": mass_err < 1e-6 and inertia_err < 1e-4 and com_err < 1e-3,
    }


def g2_hover(cfg: dict, duration: float = 5.0) -> dict:
    model, data = build_model(cfg)
    f = drone_info(model).hover_thrust_per_rotor(cfg["sim"]["gravity"])
    p0 = data.qpos[:3].copy()
    n = int(round(duration / model.opt.timestep))
    t, drift, angle = np.zeros(n), np.zeros(n), np.zeros(n)
    for i in range(n):
        data.ctrl[:] = f
        mujoco.mj_step(model, data)
        t[i] = data.time
        drift[i] = np.linalg.norm(data.qpos[:3] - p0)
        angle[i] = _quat_angle_deg(data.qpos[3:7])
    return {
        "hover_thrust_per_rotor_N": f, "max_drift_m": float(drift.max()), "max_rotation_deg": float(angle.max()),
        "passed": drift.max() < 1e-3 and angle.max() < 0.01,
        "series": {"t": t, "drift": drift, "angle": angle},
    }


def g3_free_fall(cfg: dict, duration: float = 1.0) -> dict:
    model, data = build_model(cfg)
    z0, g = data.qpos[2], cfg["sim"]["gravity"]
    n = int(round(duration / model.opt.timestep))
    t, err = np.zeros(n), np.zeros(n)
    for i in range(n):
        data.ctrl[:] = 0.0
        mujoco.mj_step(model, data)
        t[i] = data.time
        err[i] = data.qpos[2] - (z0 - 0.5 * g * data.time**2)
    return {"max_abs_err_m": float(np.abs(err).max()), "passed": np.abs(err).max() < 1e-6,
            "series": {"t": t, "err": err}}


def g4_drag(cfg: dict, duration: float = 2.0) -> dict:
    """Free fall with fluid drag off vs on: drag must oppose motion (slower fall)."""
    out = {}
    for label, on in (("off", False), ("on", True)):
        model, data = build_model(deep_merge(cfg, {"realism": {"fluid_drag": on}}))
        n = int(round(duration / model.opt.timestep))
        t, vz = np.zeros(n), np.zeros(n)
        for i in range(n):
            data.ctrl[:] = 0.0
            mujoco.mj_step(model, data)
            t[i], vz[i] = data.time, data.qvel[2]
        out[label] = (t, vz)
    vz_off, vz_on = out["off"][1][-1], out["on"][1][-1]
    # Drag force at the end of the fall from the velocity difference rate (report only)
    model_on, data_on = build_model(deep_merge(cfg, {"realism": {"fluid_drag": True}}))
    data_on.qvel[2] = -5.0
    mujoco.mj_forward(model_on, data_on)
    drag_at_5 = float(data_on.qfrc_passive[2])
    return {
        "vz_end_no_drag": float(vz_off), "vz_end_with_drag": float(vz_on),
        "drag_force_at_5mps_N": drag_at_5,
        "passed": vz_on > vz_off and drag_at_5 > 0.0,
        "series": {"t": out["off"][0], "vz_off": out["off"][1], "vz_on": out["on"][1]},
    }


def _manoeuvre(cfg: dict, dt: float, duration: float = 1.0) -> np.ndarray:
    """Open-loop manoeuvre: hover thrust plus a differential roll/yaw pulse in 0.2-0.5 s."""
    model, data = build_model(deep_merge(cfg, {"sim": {"dt": dt}}))
    f = drone_info(model).hover_thrust_per_rotor(cfg["sim"]["gravity"])
    while data.time < duration - 1e-12:
        u = np.full(4, f)
        if 0.2 <= data.time < 0.5:
            u += np.array([0.3, -0.3, -0.2, 0.2])
        data.ctrl[:] = u
        mujoco.mj_step(model, data)
    return data.qpos.copy()


def g5_timestep(cfg: dict) -> dict:
    q1, q2 = _manoeuvre(cfg, 0.001), _manoeuvre(cfg, 0.0005)
    pos_diff = float(np.linalg.norm(q1[:3] - q2[:3]))
    return {"pos_diff_m": pos_diff, "passed": pos_diff < 1e-4}


ALL_CHECKS = {"G1_properties": g1_properties, "G2_hover": g2_hover, "G3_free_fall": g3_free_fall,
              "G4_drag": g4_drag, "G5_timestep": g5_timestep}

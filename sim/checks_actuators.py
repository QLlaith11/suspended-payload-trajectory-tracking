"""Step 1.2 validation checks for the actuator stack (gates A1-A6, IMPLEMENTATION_PLAN.md)."""
from __future__ import annotations

import mujoco
import numpy as np

from .actuators import Mixer
from .config import deep_merge
from .model import build_model, drone_info, reset


def _random_feasible_wrenches(mixer: Mixer, n: int, rng: np.random.Generator) -> np.ndarray:
    """Wrenches produced by random rotor thrusts inside the bounds (feasible by construction)."""
    thrusts = rng.uniform(0.5, mixer.f_max - 0.5, size=(n, 4))
    return thrusts @ mixer.B.T


def a1_round_trip(cfg: dict, n: int = 1000) -> dict:
    model, _ = build_model(cfg)
    mixer = Mixer(drone_info(model))
    w = _random_feasible_wrenches(mixer, n, np.random.default_rng(cfg["seed"]))
    err = max(np.max(np.abs(mixer.wrench(mixer(wi)) - wi)) for wi in w)
    return {"max_err": float(err), "passed": err < 1e-9}


def _wrench_about_com(model, data, info) -> np.ndarray:
    """Actual actuator force/torque from MuJoCo's generalized actuator forces, expressed at the CoM in body axes.

    Free joint: qfrc[0:3] = force in world axes, qfrc[3:6] = torque about the body frame origin in body axes.
    """
    R = data.xmat[model.body("x2").id].reshape(3, 3)
    F_body = R.T @ data.qfrc_actuator[0:3]
    tau_origin = data.qfrc_actuator[3:6]
    tau_com = tau_origin - np.cross(info.com, F_body)
    return np.array([F_body[2], *tau_com]), F_body[:2]


def a2_physics_wrench(cfg: dict, n: int = 200) -> dict:
    """Independent check: the wrench MuJoCo actually applies equals the commanded wrench (random attitudes)."""
    model, data = build_model(cfg)
    info = drone_info(model)
    mixer = Mixer(info)
    rng = np.random.default_rng(cfg["seed"] + 1)
    err, lateral = 0.0, 0.0
    for w in _random_feasible_wrenches(mixer, n, rng):
        q = rng.standard_normal(4)
        data.qpos[3:7] = q / np.linalg.norm(q)
        data.ctrl[:] = mixer(w)
        mujoco.mj_forward(model, data)
        actual, f_xy = _wrench_about_com(model, data, info)
        err = max(err, float(np.max(np.abs(actual - w))))
        lateral = max(lateral, float(np.max(np.abs(f_xy))))
    return {"max_err": err, "max_lateral_force": lateral, "passed": err < 1e-9 and lateral < 1e-9}


def a3_rotational_response(cfg: dict) -> dict:
    model, data = build_model(cfg)
    info = drone_info(model)
    mixer = Mixer(info)
    f_hover = info.mass * cfg["sim"]["gravity"]
    worst = 0.0
    for tau in ([0.2, 0, 0], [0, 0.2, 0], [0, 0, 0.05], [0.1, -0.1, 0.02]):
        reset(model, data)
        data.ctrl[:] = mixer([f_hover, *tau])
        mujoco.mj_forward(model, data)
        expected = np.linalg.solve(info.inertia, tau)
        worst = max(worst, float(np.max(np.abs(data.qacc[3:6] - expected))))
    return {"max_err_rad_s2": worst, "passed": worst < 1e-6}


def a4_saturation(cfg: dict) -> dict:
    model, _ = build_model(cfg)
    mixer = Mixer(drone_info(model))
    f_max = mixer.f_max
    cases = {}
    # (a) too much thrust, small roll torque: roll kept exactly, thrust reduced
    w = np.array([60.0, 0.3, 0.0, 0.0]); out = mixer(w); ach = mixer.wrench(out)
    cases["a_thrust_too_high"] = abs(ach[1] - w[1]) < 1e-9 and ach[0] < w[0]
    # (b) large yaw at hover: roll/pitch and thrust kept, yaw reduced
    w = np.array([13.0, 0.1, -0.1, 5.0]); out = mixer(w); ach = mixer.wrench(out)
    cases["b_yaw_reduced_first"] = (np.allclose(ach[:3], w[:3], atol=1e-9) and 0 <= ach[3] < w[3])
    # (c) roll alone infeasible: roll scaled, everything within bounds
    w = np.array([13.0, 10.0, 0.0, 0.0]); out = mixer(w); ach = mixer.wrench(out)
    cases["c_roll_scaled"] = 0 < ach[1] < w[1]
    # (d) feasible command is untouched
    w = np.array([13.0, 0.2, -0.1, 0.05]); out = mixer(w)
    cases["d_feasible_unchanged"] = np.allclose(mixer.wrench(out), w, atol=1e-9)
    # bounds always respected (random extreme commands)
    rng = np.random.default_rng(cfg["seed"] + 2)
    bounds_ok = all(np.all((o >= -1e-12) & (o <= f_max + 1e-12))
                    for o in (mixer(rng.uniform([-10, -20, -20, -5], [100, 20, 20, 5])) for _ in range(2000)))
    cases["bounds_respected"] = bounds_ok
    return {**{k: bool(v) for k, v in cases.items()}, "passed": all(cases.values())}


def a5_motor_lag(cfg: dict, tau: float = 0.035, duration: float = 0.25) -> dict:
    out = {}
    for label, t_m in (("lag", tau), ("ideal", 0.0)):
        model, data = build_model(deep_merge(cfg, {"drone": {"motor_tau": t_m}}))
        data.ctrl[:] = 3.0
        if model.na:
            data.act[:] = 3.0
        n = int(round(duration / model.opt.timestep))
        t, f = np.zeros(n + 1), np.zeros(n + 1)
        mujoco.mj_forward(model, data)

        def rotor_force():  # motor state at the current time (actuator_force is from the start of the last step)
            return data.act[0] if model.na else data.ctrl[0]

        f[0] = rotor_force()
        for i in range(1, n + 1):
            data.ctrl[:] = 6.0
            mujoco.mj_step(model, data)
            t[i], f[i] = data.time, rotor_force()
        out[label] = (t, f)
    t, f = out["lag"]
    expected = 6.0 - 3.0 * np.exp(-t / tau)
    err = float(np.max(np.abs(f[1:] - expected[1:])))
    i_tau = int(round(tau / cfg["sim"]["dt"]))
    frac_at_tau = float((f[i_tau] - 3.0) / 3.0)
    ideal_ok = bool(np.allclose(out["ideal"][1][1:], 6.0))
    return {"max_err_N": err, "fraction_at_tau": frac_at_tau, "ideal_instant": ideal_ok,
            "passed": err < 1e-3 and abs(frac_at_tau - (1 - np.exp(-1))) < 2e-3 and ideal_ok,
            "series": {"t": t, "f_lag": f, "f_expected": expected, "f_ideal": out["ideal"][1]}}


def _manoeuvre_with_lag(cfg: dict, dt: float, duration: float = 1.0) -> np.ndarray:
    model, data = build_model(deep_merge(cfg, {"sim": {"dt": dt}, "drone": {"motor_tau": 0.035}}))
    info = drone_info(model)
    mixer = Mixer(info)
    f_hover = info.mass * cfg["sim"]["gravity"]
    while data.time < duration - 1e-12:
        tau = [0.15, -0.1, 0.03] if 0.2 <= data.time < 0.5 else [0, 0, 0]
        data.ctrl[:] = mixer([f_hover, *tau])
        mujoco.mj_step(model, data)
    return data.qpos.copy()


def a6_timestep(cfg: dict) -> dict:
    q1, q2 = _manoeuvre_with_lag(cfg, 0.001), _manoeuvre_with_lag(cfg, 0.0005)
    diff = float(np.linalg.norm(q1[:3] - q2[:3]))
    return {"pos_diff_m": diff, "passed": diff < 1e-6}


ALL_CHECKS = {"A1_round_trip": a1_round_trip, "A2_physics_wrench": a2_physics_wrench,
              "A3_rotational_response": a3_rotational_response, "A4_saturation": a4_saturation,
              "A5_motor_lag": a5_motor_lag, "A6_timestep": a6_timestep}

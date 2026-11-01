"""Step 2.2 validation checks for Paper 1's geometric controller (gates K1-K7)."""
from __future__ import annotations

import numpy as np

from . import trajectories as T
from .checks_flatness import params
from .config import load_config, deep_merge
from .controllers.geometric_paper1 import Gains, GeometricController
from .flatness import flat_state
from .simulator import Simulator
from .state import get_state, set_state

E3 = np.array([0.0, 0.0, 1.0])


def _rot(axis, angle_deg):
    axis = np.asarray(axis, float) / np.linalg.norm(axis)
    a = np.radians(angle_deg)
    K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + np.sin(a) * K + (1 - np.cos(a)) * K @ K


def _decay_rate(t, y, t0, t1):
    """Exponential decay rate from a log-linear fit of y on [t0, t1]."""
    m = (t >= t0) & (t <= t1) & (y > 0)
    return float(-np.polyfit(t[m], np.log(y[m]), 1)[0])


def _setup(cfg=None, hz=500, **gain_kw):
    cfg = cfg or load_config()
    sim = Simulator(cfg, hz)
    p = params(cfg)
    return cfg, sim, p, Gains.from_bandwidths(p, **gain_kw)


def k1_attitude_mode() -> dict:
    """Mode (a): 60 deg attitude error -> e_R -> 0; tail decay rate ~ zeta*w_R.
    Theory check under unlimited inputs: at w_R = 30 rad/s a 60 deg error saturates the X2 rotors (F3)."""
    cfg, sim, p, g = _setup(load_config(overrides={"drone": {"ideal_actuator": True}}))
    c = GeometricController(p, g, sim.control_dt)
    s0 = get_state(sim.model, sim.data)
    set_state(sim.model, sim.data, s0.x_Q, s0.v_Q, _rot([1, 1, 0], 60), np.zeros(3), np.array([0, 0, -1.0]), np.zeros(3))
    f = (p.m_Q + p.m_L) * p.g
    t, eR = [], []
    while sim.data.time < 1.5:
        s = get_state(sim.model, sim.data)
        sim.step_ctbm(c.attitude(s, f, np.eye(3)))
        t.append(sim.data.time)
        eR.append(np.linalg.norm(c.last["e_R"]))
    t, eR = np.array(t), np.array(eR)
    rate = _decay_rate(t, eR, 0.4, 1.0)
    expected = 0.9 * g.bandwidths[2]
    return {"eR_final": float(eR[-1]), "decay_rate": rate, "expected_rate": expected,
            "passed": eR[-1] < 1e-4 and abs(rate - expected) / expected < 0.25, "series": (t, eR)}


def k2_load_attitude_mode() -> dict:
    """Mode (b), paper's F_n (Eq. 28): cable starts 120 deg from q_d = -e3 (drone near free fall, high altitude)."""
    # Theory check under Paper 1's assumption of unlimited inputs (finding F3)
    cfg = load_config(overrides={"sim": {"start_altitude": 200.0}, "drone": {"ideal_actuator": True}})
    cfg, sim, p, g = _setup(cfg)
    c = GeometricController(p, g, sim.control_dt, derivative_source="numdiff")
    s0 = get_state(sim.model, sim.data)
    q0 = _rot([1, 0, 0], 120) @ np.array([0, 0, -1.0])
    set_state(sim.model, sim.data, s0.x_Q, np.zeros(3), np.eye(3), np.zeros(3), q0, np.zeros(3))
    qd = np.array([0, 0, -1.0])
    t, psi = [], []
    while sim.data.time < 4.0:
        s = get_state(sim.model, sim.data)
        sim.step_ctbm(c.load_attitude(s, qd))
        t.append(sim.data.time)
        psi.append(c.last["Psi_q"])
    t, psi = np.array(t), np.array(psi)
    # Psi_q ~ e^2/2, so sqrt(Psi_q) decays like the error. Fit window in the convergence phase, above the
    # numerical floor (~1e-6, reached after ~2 s with the 7.5 rad/s cable loop).
    rate = _decay_rate(t, np.sqrt(psi), 0.5, 1.5)
    expected = 0.9 * g.bandwidths[1]
    return {"Psi_q_final": float(psi[-1]), "error_decay_rate": rate, "expected_rate": expected,
            "passed": psi[-1] < 1e-6 and abs(rate - expected) / expected < 0.3, "series": (t, psi)}


def k3_hover_recovery() -> dict:
    """Mode (c): payload starts 1 m away from the hover reference; settling consistent with w_x."""
    cfg, sim, p, g = _setup()
    c = GeometricController(p, g, sim.control_dt)
    s0 = get_state(sim.model, sim.data)
    ref_tr = T.hover(s0.x_L)
    set_state(sim.model, sim.data, s0.x_Q + [1.0, 0, 0], np.zeros(3), np.eye(3), np.zeros(3), s0.q, np.zeros(3))
    t, err = [], []
    while sim.data.time < 6.0:
        s = get_state(sim.model, sim.data)
        r = ref_tr(s.t)
        sim.step_ctbm(c.load_position(s, r, flat_state(r, p)))
        t.append(sim.data.time)
        err.append(np.linalg.norm(get_state(sim.model, sim.data).x_L - r[0]))
    t, err = np.array(t), np.array(err)
    inside = np.where(err > 0.05)[0]
    t_settle = float(t[inside[-1] + 1]) if len(inside) else 0.0
    # linear 2nd-order estimate: 5 % settling ~ -ln(0.05*sqrt(1-zeta^2)) / (zeta*w_x)
    w, z = g.bandwidths[0], 0.9
    expected = -np.log(0.05 * np.sqrt(1 - z**2)) / (z * w)
    overshoot = float(max(0.0, -(get_state(sim.model, sim.data).x_L[0] - ref_tr(0)[0][0])))
    return {"settling_time_s": t_settle, "expected_s": float(expected), "final_err_m": float(err[-1]),
            "rel_dev": float(abs(t_settle - expected) / expected),
            "passed": abs(t_settle - expected) / expected < 0.25 and err[-1] < 1e-3, "series": (t, err)}


def tracking_cases(x0) -> dict:
    g = 9.81
    return {"T1_2s": (T.point_to_point(x0, x0 + [1.0, 0, 0], 2.0), 5.0),
            "T2_0.5g": (T.circle(x0, 1.0, 0.5 * g), 12.0),
            "T3_10s": (T.figure8(x0, 10.0), 15.0)}


def run_tracking(traj, duration, cfg=None, derivative_source="flat", **gain_kw):
    cfg, sim, p, g = _setup(cfg, **gain_kw)
    c = GeometricController(p, g, sim.control_dt, derivative_source=derivative_source)
    t, err = [], []
    while sim.data.time < duration - 1e-9:
        s = get_state(sim.model, sim.data)
        r = traj(s.t)
        sim.step_ctbm(c.load_position(s, r, flat_state(r, p)))
        t.append(sim.data.time)
        err.append(np.linalg.norm(get_state(sim.model, sim.data).x_L - traj(sim.data.time)[0]))
    return np.array(t), np.array(err)


def k4_tracking() -> dict:
    cfg = load_config()
    sim = Simulator(cfg, 500)
    x0 = get_state(sim.model, sim.data).x_L
    out, series, ok = {}, {}, True
    for name, (tr, dur) in tracking_cases(x0).items():
        t, err = run_tracking(tr, dur, cfg)
        out[f"{name}_max_err_m"] = float(err.max())
        out[f"{name}_rmse_m"] = float(np.sqrt(np.mean(err**2)))
        series[name] = (t, err)
        ok &= err.max() < 0.01
    return {**out, "passed": bool(ok), "series": series}


def k5_time_scale_separation() -> dict:
    """Faster attitude loop (smaller eps) -> smaller error on a perturbed circle (paper's singular-perturbation
    argument). Default bandwidth ordering w_x < w_q < w_R."""
    cfg = load_config(overrides={"drone": {"ideal_actuator": True}})   # theory check (F3)
    sim = Simulator(cfg, 500)
    x0 = get_state(sim.model, sim.data).x_L
    tr = T.circle(x0, 1.0, 0.5 * 9.81)
    out = {}
    for w_R in (15.0, 30.0, 60.0):
        _, err = run_tracking(tr, 12.0, cfg, w_R=w_R)
        out[f"wR{int(w_R)}_rmse_m"] = float(np.sqrt(np.mean(err**2)))
    p = params(cfg)
    bw = Gains.from_bandwidths(p).bandwidths
    monotone = out["wR15_rmse_m"] > out["wR30_rmse_m"] > out["wR60_rmse_m"]
    return {**out, "ordering_ok": bool(bw[0] < bw[1] < bw[2]), "monotone": bool(monotone),
            "passed": bool(bw[0] < bw[1] < bw[2] and monotone)}


class _WrongRateError(GeometricController):
    """Negative control: uses the cable angular-velocity error (omega - omega_d) as if it were the tangent rate
    error. This is NOT Paper 1's Eq. (18)."""

    @staticmethod
    def cable_rate_error(q, q_dot, qd, qd_dot):
        return np.cross(q, q_dot) - np.cross(qd, qd_dot)


def _swing_damping(controller_cls) -> float:
    cfg, sim, p, g = _setup()
    c = controller_cls(p, g, sim.control_dt)
    s0 = get_state(sim.model, sim.data)
    tr = T.hover(s0.x_L)
    set_state(sim.model, sim.data, s0.x_Q, np.zeros(3), np.eye(3), np.zeros(3),
              _rot([1, 0, 0], 30) @ np.array([0, 0, -1.0]), np.zeros(3))
    sw = []
    while sim.data.time < 5.0:
        s = get_state(sim.model, sim.data)
        r = tr(s.t)
        sim.step_ctbm(c.load_position(s, r, flat_state(r, p)))
        q = get_state(sim.model, sim.data).q
        if not np.all(np.isfinite(q)) or abs(sim.data.qpos[2]) > 1e3:
            return float("inf")
        sw.append(np.degrees(np.arccos(np.clip(-q[2], -1, 1))))
    return float(max(sw[-500:]))           # swing in the last second


def k6_rate_error_guard() -> dict:
    cfg, sim, p, g = _setup()
    c = GeometricController(p, g, sim.control_dt)
    rng = np.random.default_rng(0)
    tangent = True
    for _ in range(100):
        q = rng.standard_normal(3); q /= np.linalg.norm(q)
        qd = rng.standard_normal(3); qd /= np.linalg.norm(qd)
        q_dot = np.cross(rng.standard_normal(3), q)
        qd_dot = np.cross(rng.standard_normal(3), qd)
        tangent &= abs(c.cable_rate_error(q, q_dot, qd, qd_dot) @ q) < 1e-12
    correct = _swing_damping(GeometricController)
    wrong = _swing_damping(_WrongRateError)
    return {"tangent_to_S2": bool(tangent), "residual_swing_correct_deg": correct,
            "residual_swing_wrong_form_deg": wrong,
            "passed": bool(tangent and correct < 0.5 and wrong > 10 * max(correct, 0.05))}


def k7_derivative_ablation() -> dict:
    cfg = load_config()
    sim = Simulator(cfg, 500)
    x0 = get_state(sim.model, sim.data).x_L
    tr = T.circle(x0, 1.0, 0.5 * 9.81)
    out = {}
    for src in ("flat", "numdiff", "zero"):
        _, err = run_tracking(tr, 12.0, cfg, derivative_source=src)
        out[f"{src}_rmse_m"] = float(np.sqrt(np.mean(err**2)))
        out[f"{src}_max_err_m"] = float(err.max())
    return {**out, "passed": out["flat_rmse_m"] <= min(out["numdiff_rmse_m"], out["zero_rmse_m"])}


ALL_CHECKS = {"K1_attitude_mode": k1_attitude_mode, "K2_load_attitude_mode": k2_load_attitude_mode,
              "K3_hover_recovery": k3_hover_recovery, "K4_tracking": k4_tracking,
              "K5_time_scale_separation": k5_time_scale_separation, "K6_rate_error_guard": k6_rate_error_guard,
              "K7_derivative_ablation": k7_derivative_ablation}

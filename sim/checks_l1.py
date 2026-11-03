"""Step 2.6c-1 validation checks (gates R1-R8): sensor noise, action delay, CTBR rate loop, closed-loop runner,
air drag, seed reproducibility."""
from __future__ import annotations

import mujoco
import numpy as np

from . import trajectories as T
from .benchmark import run_case
from .checks_flatness import params
from .checks_geometric import _rot
from .closed_loop import run
from .config import load_config
from .controllers.geometric_paper1 import Gains, GeometricController, hat
from .controllers.rate import RateController, ctbr_from_paper1, rate_bandwidth
from .model import build_model
from .realism import CommandDelay, SensorModel
from .simulator import Simulator
from .state import get_state, set_state

L1_NOISE = {"realism": {"sensor_noise": True}}


def _so3_log(R):
    c = np.clip((np.trace(R) - 1) / 2, -1, 1)
    a = np.arccos(c)
    if a < 1e-12:
        return np.zeros(3)
    return a / (2 * np.sin(a)) * np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])


def r1_noise_statistics(n: int = 20000) -> dict:
    cfg = load_config(overrides=L1_NOISE)
    m, d = build_model(cfg)
    s = get_state(m, d)
    sens = SensorModel(cfg, seed=1)
    acc = {k: [] for k in ("x_Q", "v_Q", "att", "Omega", "x_L", "v_L", "q")}
    for _ in range(n):
        y = sens(s)
        acc["x_Q"].append(y.x_Q - s.x_Q); acc["v_Q"].append(y.v_Q - s.v_Q)
        acc["att"].append(_so3_log(s.R.T @ y.R)); acc["Omega"].append(y.Omega - s.Omega)
        acc["x_L"].append(y.x_L - s.x_L); acc["v_L"].append(y.v_L - s.v_L)
        acc["q"].append(np.degrees(np.arccos(np.clip(y.q @ s.q, -1, 1))))
    nz = cfg["realism"]["noise"]
    spec = {"x_Q": nz["pos"], "v_Q": nz["vel"], "att": np.radians(nz["att_deg"]), "Omega": nz["gyro"],
            "x_L": nz["pos"], "v_L": nz["vel"]}
    out, ok = {}, True
    for k, sd in spec.items():
        a = np.array(acc[k])
        rel = np.abs(a.std(axis=0) / sd - 1).max()
        bias = np.abs(a.mean(axis=0)).max() / sd
        out[f"{k}_std_rel_err"] = float(rel)
        ok &= rel < 0.03 and bias < 0.05
    out["q_angle_noise_rms_deg"] = float(np.sqrt(np.mean(np.array(acc["q"]) ** 2)))
    return {**out, "passed": bool(ok)}


def r2_delay() -> dict:
    d0 = CommandDelay(0.0, 0.01, 0.0)
    d1 = CommandDelay(0.01, 0.01, -1.0)
    d5 = CommandDelay(0.01, 0.002, -1.0)
    seq = list(range(10))
    o0, o1, o5 = [d0(x) for x in seq], [d1(x) for x in seq], [d5(x) for x in seq]
    try:
        CommandDelay(0.015, 0.01, 0.0)
        raised = False
    except ValueError:
        raised = True
    ok = o0 == seq and o1 == [-1] + seq[:-1] and o5 == [-1] * 5 + seq[:-5] and raised
    return {"n_ticks_100Hz": d1.n, "n_ticks_500Hz": d5.n, "non_multiple_rejected": raised, "passed": bool(ok)}


def r3_ctbr_equals_paper1_attitude_law(n: int = 200) -> dict:
    """CTBR (P attitude loop inside P rate loop) = Paper 1 Eq. 21 with Omega_c = 0 exactly; with Omega_c != 0 the
    difference is exactly the dropped feed-forward torque J(hat(Omega) R^T R_c Omega_c - R^T R_c Omega_c_dot)."""
    p = params()
    g = Gains.from_bandwidths(p)
    c = GeometricController(p, g, 0.002)
    rate = RateController(p.J, rate_bandwidth(g.bandwidths[2]))
    rng = np.random.default_rng(0)
    e0 = e1 = 0.0
    for _ in range(n):
        R = _rot(rng.standard_normal(3), rng.uniform(0, 170))
        Rc = _rot(rng.standard_normal(3), rng.uniform(0, 170))
        Om = rng.standard_normal(3)
        for Om_c, Om_cd in ((np.zeros(3), np.zeros(3)), (rng.standard_normal(3), rng.standard_normal(3))):
            M = c.attitude_moment(R, Om, Rc, Om_c, Om_cd)

            class S:
                pass
            s = S(); s.R = R
            c.last.update(R_c=Rc, Omega_c=Om_c, f=0.0)
            _, Om_cmd = ctbr_from_paper1(c, s)
            rate.integ[:] = 0
            tau = rate(Om, Om_cmd)
            ff = -p.J @ (hat(Om) @ R.T @ Rc @ Om_c - R.T @ Rc @ Om_cd)
            if not Om_c.any():
                e0 = max(e0, np.abs(tau - M).max())
            else:
                e1 = max(e1, np.abs(tau + ff - M).max())
    return {"err_Omega_c_zero_Nm": float(e0), "err_with_ff_term_Nm": float(e1), "passed": e0 < 1e-12 and e1 < 1e-12}


def _t2():
    cfg = load_config()
    x0 = get_state(*build_model(cfg)).x_L
    tr = T.circle(x0, 1.0, 0.5 * 9.81)
    return tr, tr.meta["t_ramp"] + 2 * tr.meta["period"]


def r4_runner_reproduces_step25() -> dict:
    """All realism off, CTBM at 500 Hz: closed_loop.run == benchmark.run_case (step 2.5) on T2 0.5 g."""
    tr, dur = _t2()
    m_new, L_new = run(tr, dur)
    m_old, L_old = run_case(tr, dur, 0.0, False, ideal=False)
    diff = float(np.abs(L_new["x_L"] - L_old["x_L"]).max())
    return {"max_pos_diff_m": diff, "passed": diff < 1e-12}


def r5_ctbr_tracking() -> dict:
    """L0, X2: CTBR with the outer loop at 500 Hz and at 100 Hz (D5) on T2 0.5 g; compared with CTBM."""
    tr, dur = _t2()
    out = {}
    for name, kw in (("CTBM_500", {}), ("CTBR_500", {"interface": "CTBR", "outer_hz": 500}),
                     ("CTBR_100", {"interface": "CTBR", "outer_hz": 100})):
        m, _ = run(tr, dur, **kw)
        out[f"{name}_max_err_m"] = m["payload_max_err"]
        out[f"{name}_rmse_m"] = m["payload_rmse"]
    return {**out, "passed": out["CTBR_500_max_err_m"] < 0.01}


def r6_rate_step() -> dict:
    """Ideal actuator, bare rate loop at 500 Hz: 1 rad/s roll-rate step -> first-order response, time constant
    1/w_rate (w_rate = 2 zeta w_R = 54 rad/s -> 18.5 ms) within 15 %."""
    cfg = load_config(overrides={"drone": {"ideal_actuator": True}})
    sim = Simulator(cfg, cfg["rates"]["rate_loop_hz"])
    p = params(cfg)
    w = rate_bandwidth(30.0)
    rc = RateController(p.J, w, dt=sim.control_dt)
    f = (p.m_Q + p.m_L) * p.g
    t, om = [], []
    while sim.data.time < 0.15:
        s = get_state(sim.model, sim.data)
        sim.step_ctbm(np.array([f, *rc(s.Omega, np.array([1.0, 0.0, 0.0]))]))
        t.append(sim.data.time); om.append(get_state(sim.model, sim.data).Omega[0])
    t, om = np.array(t), np.array(om)
    i = np.argmax(om >= 1 - np.exp(-1))
    tau = float(np.interp(1 - np.exp(-1), om[i - 1:i + 1], t[i - 1:i + 1]))
    expected = 1 / w
    return {"time_constant_s": tau, "expected_s": expected, "rel_err": abs(tau - expected) / expected,
            "passed": abs(tau - expected) / expected < 0.15}


def r7_drag() -> dict:
    """Fluid drag on (L1): force opposes velocity, quadratic in speed (ratio 2 m/s -> 4 m/s ~ 4), payload drag
    present (C2: free payload, so its drag is isolated) and equal to a sphere with C_D = 0.47 within 5 %."""
    out, ok = {}, True
    for part in ("drone", "payload"):
        F = {}
        for v in (2.0, 4.0):
            acc = {}
            for on in (False, True):
                cfg = load_config(overrides={"cable": {"model": "C2"}, "realism": {"fluid_drag": on}})
                m, d = build_model(cfg)
                vel = np.array([v, 0.0, 0.0])
                if part == "drone":
                    d.qvel[0:3] = vel
                else:
                    d.qvel[6:9] = vel
                mujoco.mj_forward(m, d)
                acc[on] = (d.qacc[0:3] if part == "drone" else d.qacc[6:9]).copy()
            mass = float(m.body_mass[1]) if part == "drone" else cfg["payload"]["mass"]
            F[v] = mass * (acc[True] - acc[False])
        ratio = F[4.0][0] / F[2.0][0]
        out[f"{part}_drag_x_at_2ms_N"] = float(F[2.0][0])
        out[f"{part}_drag_x_at_4ms_N"] = float(F[4.0][0])
        out[f"{part}_quadratic_ratio"] = float(ratio)
        ok &= F[2.0][0] < 0 and abs(ratio - 4) < 0.4
    sphere = 0.5 * 1.225 * 0.47 * np.pi * 0.035**2 * 4.0**2
    out["payload_sphere_CD047_at_4ms_N"] = float(sphere)
    out["payload_vs_sphere"] = float(-out["payload_drag_x_at_4ms_N"] / sphere)
    ok &= abs(out["payload_vs_sphere"] - 1.0) < 0.05
    return {**out, "passed": bool(ok)}


def r8_seeds() -> dict:
    tr = T.hover(get_state(*build_model(load_config())).x_L)
    a = run(tr, 1.0, extra=L1_NOISE, seed=3, log_tension=False)[1]["x_L"]
    b = run(tr, 1.0, extra=L1_NOISE, seed=3, log_tension=False)[1]["x_L"]
    c = run(tr, 1.0, extra=L1_NOISE, seed=4, log_tension=False)[1]["x_L"]
    same, differ = float(np.abs(a - b).max()), float(np.abs(a - c).max())
    return {"same_seed_diff_m": same, "other_seed_diff_m": differ, "passed": same == 0.0 and differ > 0.0}


ALL_CHECKS = {"R1_noise_statistics": r1_noise_statistics, "R2_delay": r2_delay,
              "R3_ctbr_equals_paper1": r3_ctbr_equals_paper1_attitude_law,
              "R4_runner_reproduces_2.5": r4_runner_reproduces_step25, "R5_ctbr_tracking": r5_ctbr_tracking,
              "R6_rate_step": r6_rate_step, "R7_drag": r7_drag, "R8_seeds": r8_seeds}

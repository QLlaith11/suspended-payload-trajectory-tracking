"""Step 1.4 validation checks: trajectories, metrics, logger (gates M1-M5)."""
from __future__ import annotations

import tempfile
from pathlib import Path

import numpy as np
import sympy as sp

from . import trajectories as T
from .config import load_config
from .logger import Logger
from .metrics import compute_metrics
from .simulator import Simulator
from .state import cable_tension, get_state

X0 = np.array([0.0, 0.0, 1.5])


def test_trajectories() -> dict:
    g = 9.81
    return {
        "circle_0.5g": T.circle(X0, 1.0, 0.5 * g),
        "figure8_5s": T.figure8(X0, 5.0),
        "p2p_1s": T.point_to_point(X0, X0 + [1.0, 0.0, 0.0], 1.0),
        "ellipse_T2": T.paper1_ellipse(1.0, 2.0, 1.5, 2.0),
    }


# ---------------------------------------------------------------- M1: exact derivatives vs a symbolic oracle
def _symbolic(traj, t_val: float) -> np.ndarray:
    """Independent oracle: build the same curve as one sympy expression (segment containing t_val), differentiate
    symbolically 6 times and evaluate."""
    t = sp.symbols("t", real=True)
    u = sp.symbols("u", real=True)
    s = sp.Poly(list(T.smoothstep13_coeffs()), u).as_expr()
    if isinstance(traj, T.PointToPoint):
        prof = s.subs(u, (t - traj.t_start) / traj.duration)
        expr = [sp.Float(a) + sp.Float(b - a) * prof for a, b in zip(traj.x0, traj.x1)]
    else:
        ph = traj.phase
        if isinstance(ph, T.SmoothRampPhase):
            S = sp.integrate(s, (u, 0, u))
            phi = ph.omega * ph.t_ramp * S.subs(u, t / ph.t_ramp) if t_val < ph.t_ramp else \
                ph.omega * ph.t_ramp * ph.S1 + ph.omega * (t - ph.t_ramp)
        else:
            phi = ph.omega * t
        expr = [sp.Float(v) + 0 * t for v in traj.x0]
        for axis, amp, mult, kind in traj.terms:
            f = {"sin": sp.sin(mult * phi), "cos": sp.cos(mult * phi), "1-cos": 1 - sp.cos(mult * phi)}[kind]
            expr[axis] = expr[axis] + amp * f
    out = np.empty((T.N_DERIV + 1, 3))
    for i, e in enumerate(expr):
        d = e
        for k in range(T.N_DERIV + 1):
            out[k, i] = float(d.subs(t, t_val).evalf(30))
            d = sp.diff(d, t)
    return out


def m1_derivatives() -> dict:
    out, ok = {}, True
    samples = {"circle_0.5g": (1.3, 5.7), "figure8_5s": (2.1, 6.4), "p2p_1s": (1.37, 1.81), "ellipse_T2": (0.3, 1.7)}
    for name, traj in test_trajectories().items():
        worst = 0.0
        for tv in samples[name]:
            ref = _symbolic(traj, tv)
            got = traj(tv)
            scale = np.maximum(np.abs(ref).max(axis=1, keepdims=True), 1.0)
            worst = max(worst, float(np.max(np.abs(got - ref) / scale)))
        out[f"{name}_max_rel_err"] = worst
        ok &= worst < 1e-10
    return {**out, "passed": ok}


# ---------------------------------------------------------------- M2: boundary conditions and smooth joins
def m2_boundaries() -> dict:
    trajs = test_trajectories()
    p = trajs["p2p_1s"]
    start, end = p(p.t_start), p(p.t_start + p.duration)
    p2p_ok = bool(np.allclose(start[0], p.x0) and np.allclose(end[0], p.x1)
                  and np.abs(start[1:]).max() < 1e-9 and np.abs(end[1:]).max() < 1e-9)
    # Continuity of derivatives 0..6 at the end of the speed ramp: the jump across [Tr - eps, Tr + eps] must
    # shrink ~linearly with eps (continuous); a true discontinuity would not shrink.
    join, ok = {}, p2p_ok
    for name in ("circle_0.5g", "figure8_5s"):
        tr = trajs[name]
        tr_ramp = tr.phase.t_ramp
        jumps = []
        for eps in (1e-5, 1e-6):
            a, b = tr(tr_ramp - eps), tr(tr_ramp + eps)
            jumps.append(np.abs(a - b).max(axis=1))       # per derivative order
        ratio = float(np.min(jumps[0] / np.maximum(jumps[1], 1e-300)))
        join[f"{name}_jump_eps1e-6"] = float(jumps[1].max())
        join[f"{name}_min_shrink_ratio"] = ratio
        ok &= ratio > 8.0                                  # ~10 expected for continuous derivatives
    return {"p2p_rest_to_rest": p2p_ok, **join, "passed": bool(ok)}


# ---------------------------------------------------------------- M3: aggressiveness parameters
def m3_aggressiveness() -> dict:
    g, out = 9.81, {}
    ok = True
    for frac in (0.25, 0.5, 1.0):
        c = T.circle(X0, 1.0, frac * g)
        ts = np.linspace(0.0, 12.0, 4001)
        acc = np.array([np.linalg.norm(c(t)[2]) for t in ts])
        steady = acc[ts > c.phase.t_ramp + 0.1]
        rel = abs(steady.mean() - frac * g) / (frac * g)
        out[f"circle_{frac}g_rel_err"] = float(rel)
        out[f"circle_{frac}g_max_over_peak"] = float(acc.max() / (frac * g))
        ok &= rel < 1e-9 and acc.max() <= frac * g * (1 + 1e-9)
    for period in (15.0, 10.0, 5.0):
        f8 = T.figure8(X0, period)
        tt = np.linspace(4.0, 9.0, 11)
        err = max(np.abs(f8(t)[0] - f8(t + period)[0]).max() for t in tt)
        out[f"figure8_{period}s_periodicity_err"] = float(err)
        ok &= err < 1e-9
    for dur in (4.0, 2.0, 1.0):
        p = T.point_to_point(X0, X0 + [1.0, 0, 0], dur)
        ts = np.linspace(p.t_start, p.t_start + dur, 20001)
        peak = max(np.linalg.norm(p(t)[2]) for t in ts)
        rel = abs(peak - p.meta["peak_acc"]) / p.meta["peak_acc"]
        out[f"p2p_{dur}s_peak_acc_g"] = float(peak / g)
        ok &= rel < 1e-3
    return {**out, "passed": bool(ok)}


# ---------------------------------------------------------------- M4: metrics on signals with known answers
def m4_metrics() -> dict:
    dt = 0.01
    t = np.arange(0.0, 10.0, dt)
    n = len(t)
    A, w = np.radians(10.0), 2 * np.pi / 1.0                # 10 deg swing, 1 s period (10 full periods)
    theta = A * np.sin(w * t)
    log = {
        "t": t,
        "ref": np.zeros((n, 7, 3)),
        "x_L": np.tile([0.1, 0.0, 0.0], (n, 1)),            # constant 0.1 m error in x
        "x_Q": np.tile([0.0, 0.0, 2.0], (n, 1)),
        "q": np.stack([np.sin(theta), np.zeros(n), -np.cos(theta)], axis=1),
        "R": np.tile(np.eye(3).reshape(9), (n, 1)),
        "T": np.full(n, 1.962),
        "rotor_cmd": np.full((n, 4), 2.0),
        "saturated": (np.arange(n) % 2).astype(float),
    }
    m = compute_metrics(log, dt)
    swing_rms_expected = np.degrees(A) / np.sqrt(2)
    checks = {
        "rmse": abs(m["payload_rmse"] - 0.1) < 1e-12 and abs(m["payload_rmse_x"] - 0.1) < 1e-12
        and m["payload_rmse_y"] == 0.0,
        "swing_max": abs(m["swing_max_deg"] - 10.0) < 1e-3,
        "swing_rms": abs(m["swing_rms_deg"] - swing_rms_expected) < 1e-3,
        "effort": abs(m["control_effort"] - 4 * 4.0 * n * dt) < 1e-9,
        "saturation": abs(m["saturation_fraction"] - 0.5) < 1e-12,
        "success": m["success"] and not m["crashed"],
    }
    # settling: error 0.2*exp(-t) after t_end = 0 -> enters 5 cm band at ln(4) = 1.386 s
    log2 = dict(log, x_L=np.stack([0.2 * np.exp(-t), np.zeros(n), np.zeros(n)], axis=1))
    ts = compute_metrics(log2, dt, t_end=0.0)["settling_time_s"]
    checks["settling"] = abs(ts - np.log(4.0)) <= dt
    # payload above the drone -> failure
    log3 = dict(log, q=log["q"] * np.array([1, 1, -1]))
    checks["payload_above_fails"] = compute_metrics(log3, dt)["payload_above_drone"] and \
        not compute_metrics(log3, dt)["success"]
    return {**{k: bool(v) for k, v in checks.items()}, "settling_time_s": ts, "passed": all(checks.values())}


# ---------------------------------------------------------------- M5: logger round trip
def m5_logger() -> dict:
    cfg = load_config()
    sim = Simulator(cfg, 500)
    traj = T.hover(get_state(sim.model, sim.data).x_L)
    log = Logger()
    M = sim.info.mass + cfg["payload"]["mass"]
    for k in range(100):
        s = get_state(sim.model, sim.data)
        w = np.array([M * 9.81, 0.02 * np.sin(k / 10), 0.0, 0.0])
        rotors = sim.step_ctbm(w)
        log.log(s, tension=cable_tension(sim.model, sim.data, 9.81), ref=traj(s.t), wrench=w, rotor_cmd=rotors,
                rotor_unconstrained=sim.mixer.unconstrained(w), ctrl_time=1e-4)
    arrays = log.arrays()
    with tempfile.TemporaryDirectory() as d:
        path = Path(d) / "log.npz"
        log.save(path)
        back = Logger.load(path)
    identical = set(back) == set(arrays) and all(np.array_equal(back[k], arrays[k]) for k in arrays)
    return {"n_fields": len(arrays), "n_samples": int(len(arrays["t"])), "identical": identical,
            "passed": identical and len(arrays["t"]) == 100}


ALL_CHECKS = {"M1_derivatives": m1_derivatives, "M2_boundaries": m2_boundaries,
              "M3_aggressiveness": m3_aggressiveness, "M4_metrics": m4_metrics, "M5_logger": m5_logger}

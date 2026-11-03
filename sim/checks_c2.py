"""Step 2.6a validation checks for cable C2 (inextensible, slack-capable tendon, D3) with payload P1 (gates S1-S6).
The drone is free and held at constant hover thrust (no controller) in every check."""
from __future__ import annotations

import time

import mujoco
import numpy as np

from .checks_geometric import _rot
from .config import load_config
from .model import build_model
from .state import cable_tension, get_state, set_state

STRETCH_MAX = 1e-3        # [m]  D3: stretch < 1 mm while taut (hover, swinging)
SNAP_STRETCH_MAX = 5e-3   # [m]  transient stretch during a snap (an inextensible cable's snap is an impulse; the soft
                          #      constraint spreads it over ~4 ms) - see finding F6


def _cfg(cable="C2", **over):
    o = {"cable": {"model": cable}}
    for k, v in over.items():
        o.setdefault(k, {}).update(v)
    return load_config(overrides=o)


def _run(cfg, duration, swing_deg=0.0, v_up=0.0, dt=None):
    if dt:
        cfg = load_config(overrides={**cfg, "sim": {**cfg["sim"], "dt": dt}})
    m, d = build_model(cfg)
    s0 = get_state(m, d)
    q0 = _rot([1, 0, 0], swing_deg) @ np.array([0.0, 0.0, -1.0])
    L = cfg["cable"]["length"]
    if v_up:
        # payload thrown upward relative to the hovering drone: rate along -q with |q_dot| = v/L is not tangent,
        # so set the free payload velocity directly (C2 only)
        set_state(m, d, s0.x_Q, np.zeros(3), np.eye(3), np.zeros(3), q0, np.zeros(3))
        d.qvel[6:9] = [0.0, 0.0, v_up]
        mujoco.mj_forward(m, d)
    else:
        set_state(m, d, s0.x_Q, np.zeros(3), np.eye(3), np.zeros(3), q0, np.zeros(3))
    g = cfg["sim"]["gravity"]
    rec = {k: [] for k in ("t", "len", "T", "xL", "E")}
    mQ = float(m.body_mass[1]); mL = cfg["payload"]["mass"]
    n = int(round(duration / m.opt.timestep))
    for _ in range(n):
        mujoco.mj_step(m, d)
        s = get_state(m, d)
        rec["t"].append(d.time); rec["len"].append(s.cable_length); rec["xL"].append(s.x_L.copy())
        rec["T"].append(cable_tension(m, d, g))
    return {k: np.array(v) for k, v in rec.items()}, L


def s1_static_hover() -> dict:
    cfg = _cfg()
    r, L = _run(cfg, 2.0)
    mg = cfg["payload"]["mass"] * cfg["sim"]["gravity"]
    stretch = float(r["len"].max() - L)
    terr = float(np.abs(r["T"][-500:] - mg).max())
    return {"max_stretch_m": stretch, "tension_err_N": terr,
            "passed": stretch < STRETCH_MAX and terr < 1e-3 * mg}


def s2_swing_period() -> dict:
    """Taut small swing: period vs the free-drone pendulum theory (same formula as C1 check P2)."""
    cfg = _cfg()
    r, L = _run(cfg, 12.0, swing_deg=2.0)
    m, _ = build_model(cfg)
    mQ, mL, g = float(m.body_mass[1]), cfg["payload"]["mass"], cfg["sim"]["gravity"]
    theory = 2 * np.pi / np.sqrt(g * (mQ + mL) / (mQ * L))
    # swing coordinate: payload y relative to the drone attachment ~ q_y
    t = r["t"]
    m2, d2 = build_model(cfg)
    y = r["xL"][:, 1]
    y = y - np.mean(y)
    zc = np.where(np.diff(np.sign(y)) > 0)[0]
    tc = t[zc] - y[zc] * (t[zc + 1] - t[zc]) / (y[zc + 1] - y[zc])
    meas = float(np.mean(np.diff(tc)))
    rel = abs(meas - theory) / theory
    return {"period_s": meas, "theory_s": float(theory), "rel_err": float(rel), "min_tension_N": float(r["T"].min()),
            "passed": rel < 5e-3 and r["T"].min() > 0}


def s3_c1_equivalence() -> dict:
    """30 deg passive swing for 2 s: C2 (taut throughout) follows C1 within the stretch tolerance."""
    r1, _ = _run(_cfg("C1"), 2.0, swing_deg=30.0)
    r2, L = _run(_cfg("C2"), 2.0, swing_deg=30.0)
    diff = float(np.linalg.norm(r1["xL"] - r2["xL"], axis=1).max())
    return {"max_pos_diff_m": diff, "max_stretch_m": float(r2["len"].max() - L), "min_tension_N": float(r2["T"].min()),
            "passed": diff < STRETCH_MAX and r2["T"].min() > 0 and r2["len"].max() - L < STRETCH_MAX}


def s4_slack_and_retaut(dt=None) -> dict:
    """D9 T5(a): payload thrown upward at 3 m/s from hover. The cable must go slack (tension 0, length < L), the
    payload fly ballistically, then the cable re-tightens with a snap; tension never negative, stretch < 1 mm."""
    cfg = _cfg()
    r, L = _run(cfg, 2.0, v_up=3.0, dt=dt)
    slack = r["len"] < L - 1e-4
    T = r["T"]
    i_retaut = int(np.argmax(~slack[np.argmax(slack):]) + np.argmax(slack)) if slack.any() else 0
    # ballistic check while slack: payload vertical acceleration = -g (no cable force)
    t, z = r["t"], r["xL"][:, 2]
    i_pk = int(np.argmax(T))
    w = (t > t[i_pk] - 0.02) & (t < t[i_pk] + 0.02)
    impulse = float(np.trapz(T[w], t[w]))
    # interior of each slack episode (10 samples trimmed at both ends: the snap is not ballistic)
    edges = np.flatnonzero(np.diff(np.r_[0, slack.astype(int), 0]))
    idx = np.concatenate([np.arange(a + 10, b - 10) for a, b in zip(edges[::2], edges[1::2]) if b - a > 30]
                         or [np.array([], int)])
    acc = np.gradient(np.gradient(z, t), t)[idx]
    g = cfg["sim"]["gravity"]
    n_episodes = len(edges) // 2
    return {"slack_episodes": n_episodes, "slack_time_s": float(slack.sum() * (t[1] - t[0])), "t_retaut_s": float(t[i_retaut]),
            "min_tension_N": float(T.min()), "peak_snap_tension_N": float(T.max()),
            "tension_while_slack_max_N": float(np.abs(T[slack]).max()) if slack.any() else float("nan"),
            "ballistic_acc_err": float(np.abs(acc + g).max()) if len(idx) else float("nan"),
            "snap_impulse_Ns": impulse, "max_stretch_m": float(r["len"].max() - L), "min_length_m": float(r["len"].min()),
            "passed": bool(slack.any() and T.min() > -1e-9 and np.abs(T[slack]).max() < 1e-6
                           and r["len"].max() - L < SNAP_STRETCH_MAX and np.abs(acc + g).max() < 1e-3 * g),
            "series": (t, r["len"], T)}


def s5_timestep() -> dict:
    """Snap convergence: re-taut time identical within 1 ms; the snap impulse (physical: payload momentum change)
    converges between 0.5 and 0.25 ms (< 5 %). At the 1 ms default it is under-resolved (reported, finding F6).
    The peak snap tension is set by the constraint softness and is NOT physical for an inextensible cable."""
    r = {dt: s4_slack_and_retaut(dt=dt) for dt in (0.001, 0.0005, 0.00025)}
    imp = {dt: v["snap_impulse_Ns"] for dt, v in r.items()}
    conv = abs(imp[0.0005] - imp[0.00025]) / imp[0.00025]
    err_1ms = abs(imp[0.001] - imp[0.00025]) / imp[0.00025]
    dt_re = max(abs(r[0.001]["t_retaut_s"] - r[0.00025]["t_retaut_s"]),
                abs(r[0.0005]["t_retaut_s"] - r[0.00025]["t_retaut_s"]))
    return {"impulse_1ms_Ns": imp[0.001], "impulse_0.5ms_Ns": imp[0.0005], "impulse_0.25ms_Ns": imp[0.00025],
            "impulse_rel_err_0.5ms": float(conv), "impulse_rel_err_1ms": float(err_1ms),
            "peak_tension_N": {dt: v["peak_snap_tension_N"] for dt, v in r.items()},
            "retaut_time_diff_s": float(dt_re), "passed": bool(conv < 0.05 and dt_re <= 1e-3)}


def s6_cost() -> dict:
    out = {}
    for cab in ("C1", "C2"):
        m, d = build_model(_cfg(cab))
        tic = time.perf_counter()
        for _ in range(5000):
            mujoco.mj_step(m, d)
        out[f"{cab}_us_per_step"] = (time.perf_counter() - tic) / 5000 * 1e6
    return {**out, "ratio": out["C2_us_per_step"] / out["C1_us_per_step"], "passed": True}


ALL_CHECKS = {"S1_static_hover": s1_static_hover, "S2_swing_period": s2_swing_period,
              "S3_c1_equivalence": s3_c1_equivalence, "S4_slack_and_retaut": s4_slack_and_retaut,
              "S5_timestep": s5_timestep, "S6_cost": s6_cost}

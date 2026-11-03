"""Step 2.5 (E1.3): benchmark subset T0-T3 (D9) for Paper 1's geometric controller at nominal settings
(X2, C1 0.5 m, P1 0.2 kg, L0, gains 1.5/7.5/30 rad/s). L0 is deterministic -> one run per case (seeds start with
L1 noise in step 2.6). Every case is run on the X2 and with the ideal actuator, and pre-checked for flatness
feasibility (rotor thrusts within 0-13 N along the reference) so that X2 failures can be attributed.
"""
from __future__ import annotations

import time

import numpy as np

from . import trajectories as T
from .actuators import Mixer
from .checks_flatness import params
from .checks_geometric import _rot
from .config import deep_merge, load_config
from .controllers.geometric_paper1 import Gains, GeometricController
from .flatness import flat_state, wrench
from .logger import Logger
from .metrics import compute_metrics
from .model import PAYLOAD_BODY
from .simulator import Simulator
from .state import cable_tension, get_state, set_state

G = 9.81
GUST_START, GUST_LEN = 2.0, 0.5                        # D9 / B5: lateral gust on the payload for 0.5 s
GUST_ACC = 0.25 * 9.81                                 # D9 revision (step 2.5): main gust = 0.25 g x payload mass
GUST_SEVERE_N = 2.0                                    # original B5 value, kept as the "severe" level


def cases(x0) -> dict:
    """name -> (trajectory, duration, initial swing [deg], gust)."""
    c = {"T0_swing15": (T.hover(x0), 10.0, 15.0, False),
         "T0_swing30": (T.hover(x0), 10.0, 30.0, False),
         "T0_gust": (T.hover(x0), 10.0, 0.0, "main"),
         "T0_gust_severe": (T.hover(x0), 10.0, 0.0, "severe")}
    for d in (4.0, 2.0, 1.0):
        tr = T.point_to_point(x0, x0 + [1.0, 0, 0], d)
        c[f"T1_{d:g}s"] = (tr, tr.t_end + 5.0, 0.0, False)
    for a in (0.25, 0.5, 1.0):
        tr = T.circle(x0, 1.0, a * G)
        c[f"T2_{a:g}g"] = (tr, tr.meta["t_ramp"] + 3 * tr.meta["period"], 0.0, False)
    for P in (15.0, 10.0, 5.0):
        tr = T.figure8(x0, P)
        c[f"T3_{P:g}s"] = (tr, tr.meta["t_ramp"] + 2 * P, 0.0, False)
    return c


def flat_feasible(traj, duration, p, mixer, dt=0.01) -> dict:
    rot = np.array([mixer.unconstrained(wrench(flat_state(traj(t), p))) for t in np.arange(0.0, duration, dt)])
    return {"feasible": bool(rot.min() >= mixer.f_min and rot.max() <= mixer.f_max),
            "flat_rotor_min": float(rot.min()), "flat_rotor_max": float(rot.max())}


def run_case(traj, duration, swing0, gust, ideal: bool, extra: dict | None = None) -> tuple[dict, dict]:
    over = deep_merge(extra or {}, {"drone": {"ideal_actuator": True}} if ideal else {})
    cfg = load_config(overrides=over)
    sim = Simulator(cfg, cfg["rates"]["rate_loop_hz"])
    p = params(cfg)
    c = GeometricController(p, Gains.from_bandwidths(p), sim.control_dt)
    s0 = get_state(sim.model, sim.data)
    if swing0:
        set_state(sim.model, sim.data, s0.x_Q, np.zeros(3), np.eye(3), np.zeros(3),
                  _rot([1, 0, 0], swing0) @ np.array([0, 0, -1.0]), np.zeros(3))
    bid = sim.model.body(PAYLOAD_BODY).id
    gust_n = {"main": GUST_ACC * p.m_L, "severe": GUST_SEVERE_N}.get(gust, 0.0)
    log, qref, xQref = Logger(), [], []
    while sim.data.time < duration - 1e-9:
        s = get_state(sim.model, sim.data)
        r = traj(s.t)
        tic = time.perf_counter()                      # controller time includes the flatness feed-forward
        fs = flat_state(r, p)
        cmd = c.load_position(s, r, fs)
        ctime = time.perf_counter() - tic
        want = sim.mixer.unconstrained(cmd)
        on = gust and GUST_START <= s.t < GUST_START + GUST_LEN
        sim.data.xfrc_applied[bid, :3] = [0.0, gust_n, 0.0] if on else 0.0
        log.log(s, tension=cable_tension(sim.model, sim.data, p.g), ref=r, wrench=cmd,
                rotor_cmd=sim.mixer(cmd), rotor_unconstrained=want, ctrl_time=ctime)
        sim.step_rotor(log._rec["rotor_cmd"][-1])
        qref.append(fs.q); xQref.append(fs.x_Q)
        if not np.all(np.isfinite(s.x_Q)) or abs(s.x_Q[2]) > 1e3:
            break
    L = log.arrays()
    t_end = traj.t_end if traj.t_end is not None else (GUST_START + GUST_LEN if gust else None)
    m = compute_metrics(L, sim.control_dt, t_end=t_end, q_ref=np.array(qref), x_Q_ref=np.array(xQref),
                        pendulum_period=2 * np.pi * np.sqrt(p.l / p.g))
    return m, L


def run_all(names=None, extra: dict | None = None) -> dict:
    cfg = load_config(overrides=extra or {})
    sim = Simulator(cfg, cfg["rates"]["rate_loop_hz"])
    x0 = get_state(sim.model, sim.data).x_L
    p, mixer = params(cfg), Mixer(sim.info)
    out = {}
    for name, (tr, dur, sw, gust) in cases(x0).items():
        if names and name not in names:
            continue
        feas = flat_feasible(tr, dur, p, mixer)
        mx, Lx = run_case(tr, dur, sw, gust, ideal=False, extra=extra)
        mi, Li = run_case(tr, dur, sw, gust, ideal=True, extra=extra)
        out[name] = {"feasibility": feas, "x2": mx, "ideal": mi, "log_x2": Lx, "log_ideal": Li}
    finished = all(np.all(np.isfinite(v["log_x2"]["x_L"])) and np.all(np.isfinite(v["log_ideal"]["x_L"]))
                   for v in out.values())
    return {"cases": out, "passed": bool(finished)}


TABLE_COLS = [("payload_rmse", "RMSE [m]", "{:.2e}"), ("payload_max_err", "max err [m]", "{:.2e}"),
              ("swing_max_deg", "max swing [°]", "{:.1f}"), ("dynamic_swing_max_deg", "dyn swing [°]", "{:.2f}"),
              ("swing_residual_deg", "resid swing [°]", "{:.2f}"), ("settling_time_s", "settle [s]", "{:.2f}"),
              ("tension_min_N", "T min [N]", "{:.2f}"), ("saturation_fraction", "sat", "{:.2f}"),
              ("tilt_max_deg", "max tilt [°]", "{:.1f}"), ("success", "success", "{}"),
              ("ctrl_time_mean_ms", "ctrl [ms]", "{:.3f}")]


def table(res: dict) -> tuple[str, str]:
    """Markdown and CSV results tables."""
    hdr = ["case", "actuator", "flat-feasible"] + [h for _, h, _ in TABLE_COLS]
    md = ["| " + " | ".join(hdr) + " |", "|" + "---|" * len(hdr)]
    csv = [",".join(k for k in ["case", "actuator", "flat_feasible"] + [k for k, _, _ in TABLE_COLS])]
    for name, v in res["cases"].items():
        for act in ("x2", "ideal"):
            m = v[act]
            cells = [fmt.format(m[k]) if k in m and m[k] is not None else "–" for k, _, fmt in TABLE_COLS]
            cells = ["–" if c == "nan" else c for c in cells]
            md.append("| " + " | ".join([name, act, "yes" if v["feasibility"]["feasible"] else "**no**"] + cells) + " |")
            csv.append(",".join([name, act, str(v["feasibility"]["feasible"])] + [str(m.get(k, "")) for k, _, _ in TABLE_COLS]))
    return "\n".join(md), "\n".join(csv)

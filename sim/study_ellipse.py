"""Step 2.3 (E1.1): Paper 1's flatness study on the ellipse x_L = (A_x sin 2πt/T0, A_y cos 2πt/T0, A_z), ψ = 0.

Part A  reproduce Paper 1 Fig. 3: drone trajectories required for the same load ellipse, T0 = 10 ... 1 s (flat map).
        Gate: load path exact; required tilt and peak torque grow as T0 decreases. (The drone's circle SHRINKS as T0
        decreases: the load swings outward, the drone flies inside it.)
Part B  X2 feasibility (new result): the shortest period T0* for which the flatness-based rotor thrusts stay within
        the X2's [0, 13] N, found by bisection; size sweep over ellipse scale.
Part C  closed loop (Paper 1 controller, mode c) on the X2 vs the ideal actuator, 3 full periods per T0, starting
        exactly on the reference flat state.
Agreed sizes: main 0.5 x 1.0 m (A_x < A_y as in the paper), second 1.0 x 2.0 m; A_z = payload hover height.
"""
from __future__ import annotations

import numpy as np

from . import trajectories as T
from .actuators import Mixer
from .checks_flatness import params
from .config import load_config
from .controllers.geometric_paper1 import Gains, GeometricController
from .flatness import FlatParams, flat_state, wrench
from .model import build_model, drone_info
from .simulator import Simulator
from .state import get_state, set_state

SIZES = {"main_0.5x1.0": (0.5, 1.0), "second_1.0x2.0": (1.0, 2.0)}
PERIODS_A = (10.0, 5.0, 4.0, 3.0, 2.5, 2.0, 1.5, 1.0)
PERIODS_C = (10.0, 5.0, 4.0, 3.0, 2.5, 2.0, 1.5)
N_PERIODS = 3
SAMPLES_PER_PERIOD = 400


def hover_height(cfg=None) -> float:
    cfg = cfg or load_config()
    return float(cfg["sim"]["start_altitude"] - cfg["cable"]["length"])


def ellipse(size, T0, az):
    return T.paper1_ellipse(size[0], size[1], az, T0)


def flat_profile(traj, T0: float, p: FlatParams, mixer: Mixer, n: int = SAMPLES_PER_PERIOD) -> dict:
    """Flat-map quantities over one period (periodic, so one period is enough)."""
    ts = np.linspace(0.0, T0, n, endpoint=False)
    xQ, xL, f, M, rot, tilt = [], [], [], [], [], []
    for t in ts:
        fs = flat_state(traj(t), p)
        xQ.append(fs.x_Q); xL.append(fs.x_L); f.append(fs.f); M.append(fs.M)
        tilt.append(np.degrees(np.arccos(np.clip(fs.R[2, 2], -1, 1))))
        rot.append(mixer.unconstrained(wrench(fs)))
    xQ, xL, M, rot = map(np.array, (xQ, xL, M, rot))
    return {"t": ts, "x_Q": xQ, "x_L": xL, "f": np.array(f), "M": M, "rotors": rot,
            "rotor_max": float(rot.max()), "rotor_min": float(rot.min()),
            "M_max": float(np.linalg.norm(M, axis=1).max()), "f_max": float(np.max(f)),
            "f_min": float(np.min(f)), "tilt_max_deg": float(np.max(tilt)),
            "quad_radius_max": float(np.linalg.norm(xQ[:, :2], axis=1).max())}


def feasible(prof: dict, mixer: Mixer) -> bool:
    return prof["rotor_min"] >= mixer.f_min and prof["rotor_max"] <= mixer.f_max


def critical_period(size, p, mixer, az, lo=0.5, hi=20.0, tol=1e-3) -> float:
    """Smallest T0 with all rotor thrusts within limits (feasibility is monotone in T0 over this range)."""
    ok = lambda T0: feasible(flat_profile(ellipse(size, T0, az), T0, p, mixer, 200), mixer)
    if not ok(hi):
        return float("nan")
    while hi - lo > tol:
        mid = 0.5 * (lo + hi)
        lo, hi = (lo, mid) if ok(mid) else (mid, hi)
    return hi


# --------------------------------------------------------------------------------------------- Part A
def part_a(cfg=None) -> dict:
    cfg = cfg or load_config()
    p, info = params(cfg), drone_info(build_model(cfg)[0])
    mixer, az = Mixer(info), hover_height(cfg)
    out = {}
    for name, size in SIZES.items():
        rows = {}
        for T0 in PERIODS_A:
            rows[T0] = flat_profile(ellipse(size, T0, az), T0, p, mixer)
        Ts = np.array(PERIODS_A)
        Mmax = np.array([rows[T0]["M_max"] for T0 in PERIODS_A])
        tilt = np.array([rows[T0]["tilt_max_deg"] for T0 in PERIODS_A])
        # local log-log slope of the peak torque between consecutive periods
        slopes = np.diff(np.log(Mmax)) / np.diff(np.log(Ts))
        load_same = max(np.abs(rows[T0]["x_L"][:, :2] / size - np.c_[np.sin(2 * np.pi * rows[T0]["t"] / T0),
                                                                       np.cos(2 * np.pi * rows[T0]["t"] / T0)]).max()
                        for T0 in PERIODS_A)
        out[name] = {"rows": rows, "torque_slopes": slopes, "load_path_err": float(load_same),
                     "tilt_monotone": bool(np.all(np.diff(tilt) > 0)),             # T0 decreasing -> grows
                     "torque_monotone": bool(np.all(np.diff(Mmax) > 0))}
    passed = all(v["load_path_err"] < 1e-12 and v["tilt_monotone"] and v["torque_monotone"]
                 for v in out.values())
    return {"sizes": out, "passed": passed}


# --------------------------------------------------------------------------------------------- Part B
def part_b(cfg=None, scales=(0.25, 0.5, 0.75, 1.0, 1.5, 2.0, 3.0)) -> dict:
    """T0* for both agreed sizes (pre-check: 2.0-2.5 s and 2.5-3.0 s) and a size sweep (A_x : A_y = 1 : 2)."""
    cfg = cfg or load_config()
    p, info = params(cfg), drone_info(build_model(cfg)[0])
    mixer, az = Mixer(info), hover_height(cfg)
    crit = {name: critical_period(size, p, mixer, az) for name, size in SIZES.items()}
    sweep = {s: critical_period((0.5 * s, 1.0 * s), p, mixer, az) for s in scales}
    pre_check = 2.0 < crit["main_0.5x1.0"] < 2.5 and 2.5 < crit["second_1.0x2.0"] < 3.0
    # peak speed / acceleration of the load at T0* (the speed limit that the X2 imposes)
    load_at_crit = {}
    for name, size in SIZES.items():
        w = 2 * np.pi / crit[name]
        load_at_crit[name] = {"v_peak": float(max(size) * w), "a_peak": float(max(size) * w**2)}
    sweep_monotone = bool(np.all(np.diff([sweep[s] for s in scales]) > 0))
    return {"T0_crit": crit, "sweep": sweep, "load_at_crit": load_at_crit,
            "pre_check_ok": bool(pre_check), "sweep_monotone": sweep_monotone,
            "passed": bool(pre_check and sweep_monotone)}


# --------------------------------------------------------------------------------------------- Part C
def run_closed_loop(size, T0, ideal: bool, cfg=None) -> dict:
    over = {"drone": {"ideal_actuator": True}} if ideal else {}
    cfg = load_config(overrides=over) if cfg is None else cfg
    sim = Simulator(cfg, cfg["rates"]["rate_loop_hz"])
    p = params(cfg)
    c = GeometricController(p, Gains.from_bandwidths(p), sim.control_dt)
    traj = ellipse(size, T0, hover_height(cfg))
    fs0 = flat_state(traj(0.0), p)
    set_state(sim.model, sim.data, fs0.x_Q, fs0.v_Q, fs0.R, fs0.Omega, fs0.q, fs0.q_dot)
    t_end = N_PERIODS * T0
    t, err, sat, rot = [], [], [], []
    diverged = False
    while sim.data.time < t_end - 1e-9:
        s = get_state(sim.model, sim.data)
        r = traj(s.t)
        cmd = c.load_position(s, r, flat_state(r, p))
        want = sim.mixer.unconstrained(cmd)
        got = sim.step_ctbm(cmd)
        sat.append(float(np.abs(got - want).max() > 1e-6))
        rot.append(got)
        x = get_state(sim.model, sim.data).x_L
        e = float(np.linalg.norm(x - traj(sim.data.time)[0]))
        t.append(sim.data.time); err.append(e)
        if not np.isfinite(e) or e > 10.0:
            diverged = True
            break
    t, err, rot = np.array(t), np.array(err), np.array(rot)
    return {"t": t, "err": err, "rotors": rot, "diverged": diverged,
            "rmse": float(np.sqrt(np.mean(err**2))), "max_err": float(err.max()),
            "sat_frac": float(np.mean(sat))}


def part_c(T0_crit: dict, periods=PERIODS_C) -> dict:
    """Gates: ideal actuator tracks every T0 >= 2 s (<= 1 cm max error); on the X2, every T0 with margin
    (T0 >= 1.2 T0*) tracks with no saturation and the same error as the ideal actuator (within 1e-4 m).
    T0 = 1.5 s is reported, not gated: the required attitude rate (~62 rad/s) exceeds the 30 rad/s attitude loop,
    so even the ideal actuator loses the reference (controller bandwidth limit, finding F4)."""
    out, ok = {}, True
    for name, size in SIZES.items():
        res = {}
        for T0 in periods:
            ide = run_closed_loop(size, T0, ideal=True)
            x2 = run_closed_loop(size, T0, ideal=False)
            res[T0] = {"ideal": ide, "x2": x2}
            if T0 >= 2.0:
                ok &= (not ide["diverged"]) and ide["max_err"] < 0.01
            if T0 >= 1.2 * T0_crit[name]:
                ok &= x2["sat_frac"] == 0.0 and abs(x2["max_err"] - ide["max_err"]) < 1e-4
        out[name] = res
    return {"sizes": out, "passed": bool(ok)}

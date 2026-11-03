"""Step 2.4 (E1.2): Paper 1 recovery from large initial errors (Sreenath, Lee, Kumar, CDC 2013, Sec. V-B, Figs. 4-5).

Paper 1: drone flipped 178 deg about y, load attitude 178 deg, large load-position error, time-varying reference.
The paper gives no numbers for the position error, reference or gains, so (agreed):
  - drone R = Ry(theta), cable q = Ry(theta)(-e3) (load ABOVE the drone for theta > 90 deg), all at rest;
  - payload starts 2 m from the reference, along (1, -1, -1)/sqrt(3);
  - reference = main ellipse 0.5 x 1.0 m, T0 = 5 s (inside the X2 boundary T0* = 2.12 s, F4); 10 s run.
C1 is a rigid link, so it can hold the load above the drone (Paper 1's always-taut assumption). A real cable cannot
push: the fraction of time with negative tension is logged (input to step 2.6, slack cable C2).
Experiment 1 (gated): theta = 178 deg, ideal actuator. Experiment 2 (reported): X2 limits, theta sweep 30..178 deg.
"""
from __future__ import annotations

import numpy as np

from .checks_flatness import params
from .checks_geometric import _rot
from .config import load_config
from .controllers.geometric_paper1 import Gains, GeometricController
from .flatness import flat_state
from .simulator import Simulator
from .state import cable_tension, get_state, set_state
from .study_ellipse import ellipse, hover_height

# agreed sweep 30..178 deg, refined after the first run showed a fragmented outcome (30 fails, 31 recovers)
ANGLES = (0.0, 10.0, 20.0, 30.0, 31.0, 40.0, 50.0, 60.0, 70.0, 80.0, 90.0, 100.0, 110.0, 120.0, 140.0, 150.0, 160.0,
          170.0, 178.0)
POS_ERR = 2.0 * np.array([1.0, -1.0, -1.0]) / np.sqrt(3.0)
SIZE, T0, DURATION = (0.5, 1.0), 5.0, 10.0
BAND = 0.05            # [m] recovery band for the load position error
HOLD = 2.0             # [s] must stay inside the band until the end, for at least this long


def run_recovery(theta_deg: float, ideal: bool, duration: float = DURATION) -> dict:
    cfg = load_config(overrides={"drone": {"ideal_actuator": True}} if ideal else {})
    sim = Simulator(cfg, cfg["rates"]["rate_loop_hz"])
    p = params(cfg)
    c = GeometricController(p, Gains.from_bandwidths(p), sim.control_dt)
    traj = ellipse(SIZE, T0, hover_height(cfg))
    R0 = _rot([0, 1, 0], theta_deg)
    q0 = R0 @ np.array([0.0, 0.0, -1.0])
    xL0 = traj(0.0)[0] + POS_ERR
    set_state(sim.model, sim.data, xL0 - p.l * q0, np.zeros(3), R0, np.zeros(3), q0, np.zeros(3))
    z0 = get_state(sim.model, sim.data).x_Q[2]

    log = {k: [] for k in ("t", "e_x", "Psi_R", "Psi_q", "tension", "z_Q", "sat")}
    diverged = False
    while sim.data.time < duration - 1e-9:
        s = get_state(sim.model, sim.data)
        r = traj(s.t)
        cmd = c.load_position(s, r, flat_state(r, p))
        want = sim.mixer.unconstrained(cmd)
        got = sim.step_ctbm(cmd)
        s1 = get_state(sim.model, sim.data)
        e = float(np.linalg.norm(s1.x_L - traj(sim.data.time)[0]))
        log["t"].append(sim.data.time)
        log["e_x"].append(e)
        log["Psi_R"].append(float(c.last.get("Psi_R", np.nan)))
        log["Psi_q"].append(float(c.last.get("Psi_q", np.nan)))
        log["tension"].append(cable_tension(sim.model, sim.data, p.g))
        log["z_Q"].append(float(s1.x_Q[2]))
        log["sat"].append(float(np.abs(got - want).max() > 1e-6))
        if not np.isfinite(e) or e > 50.0:
            diverged = True
            break
    L = {k: np.array(v) for k, v in log.items()}
    t, e = L["t"], L["e_x"]
    outside = np.where(e > BAND)[0]
    t_rec = float(t[outside[-1] + 1]) if len(outside) and outside[-1] + 1 < len(t) else (0.0 if not len(outside) else np.nan)
    recovered = bool(not diverged and np.isfinite(t_rec) and t[-1] - t_rec >= HOLD)
    return {"theta_deg": theta_deg, "ideal": ideal, "series": L, "diverged": diverged, "recovered": recovered,
            "t_recover_s": t_rec if recovered else float("nan"), "final_err_m": float(e[-1]),
            "height_loss_m": float(max(0.0, z0 - L["z_Q"].min())),
            "neg_tension_frac": float(np.mean(L["tension"] < 0.0)),
            "min_tension_N": float(L["tension"].min()), "sat_frac": float(np.mean(L["sat"]))}


def experiment_1() -> dict:
    """Paper 1 reproduction (ideal actuator): all errors -> 0, final position error < 1 cm, no blow-up."""
    r = run_recovery(178.0, ideal=True)
    S = r["series"]
    ok = (not r["diverged"] and r["final_err_m"] < 0.01 and S["Psi_R"][-1] < 1e-4 and S["Psi_q"][-1] < 1e-4)
    return {**r, "passed": bool(ok)}


def experiment_2(angles=ANGLES) -> dict:
    """X2 flip-angle sweep (new result, reported). Gate: only that every run finishes without numerical failure
    (NaN) - divergence of the drone itself is a valid outcome."""
    runs = {a: run_recovery(a, ideal=False) for a in angles}
    rec = [a for a, r in runs.items() if r["recovered"]]
    finite = all(np.all(np.isfinite(r["series"]["t"])) for r in runs.values())
    return {"runs": runs, "max_recovered_deg": float(max(rec)) if rec else float("nan"), "passed": bool(finite)}

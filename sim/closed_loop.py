"""Closed-loop runner with the full actuation/sensing stack (D5, D8, D10), step 2.6c.

    true state --SensorModel--> measured state --outer controller (outer_hz)--> command --CommandDelay-->
        CTBM: (f, M) held                                          -> mixer -> rotors (motor lag in MuJoCo)
        CTBR: (f, Omega_cmd) held -> RateController (rate_loop_hz)   -> mixer -> rotors

The rate loop uses the measured gyro. Metrics are computed on the TRUE state. With every realism option off,
interface CTBM and outer_hz = 500 this is identical to benchmark.run_case (step 2.5).
"""
from __future__ import annotations

import time

import numpy as np

from .checks_flatness import params
from .checks_geometric import _rot
from .config import deep_merge, load_config
from .controllers.geometric_paper1 import Gains, GeometricController
from .controllers.rate import RateController, ctbr_from_paper1, rate_bandwidth
from .flatness import flat_state
from .logger import Logger
from .metrics import compute_metrics
from .model import PAYLOAD_BODY
from .realism import CommandDelay, SensorModel
from .simulator import Simulator
from .state import cable_tension, get_state, set_state


def run(traj, duration: float, extra: dict | None = None, interface: str = "CTBM", outer_hz: float | None = None,
        seed: int = 0, swing0: float = 0.0, gust_n: float = 0.0, gust_window=(2.0, 2.5),
        bandwidths=(1.5, 7.5, 30.0), ideal: bool = False, log_tension: bool = True) -> tuple[dict, dict]:
    over = deep_merge(extra or {}, {"drone": {"ideal_actuator": True}} if ideal else {})
    cfg = load_config(overrides=over)
    inner_hz = cfg["rates"]["rate_loop_hz"]
    outer_hz = outer_hz or (inner_hz if interface == "CTBM" else cfg["rates"]["outer_loop_hz"])
    ratio = int(round(inner_hz / outer_hz))
    if abs(ratio * outer_hz - inner_hz) > 1e-9:
        raise ValueError("outer rate must divide the rate-loop rate")
    sim = Simulator(cfg, inner_hz)
    p = params(cfg)
    gains = Gains.from_bandwidths(p, *bandwidths)
    outer_dt = ratio * sim.control_dt
    ctrl = GeometricController(p, gains, outer_dt)
    rate = RateController(p.J, rate_bandwidth(bandwidths[2]), dt=sim.control_dt)
    sensor = SensorModel(cfg, seed)
    hover = np.array([(p.m_Q + p.m_L) * p.g, 0.0, 0.0, 0.0])
    delay = CommandDelay(cfg["realism"]["action_delay"], outer_dt, hover.copy())

    s0 = get_state(sim.model, sim.data)
    if swing0:
        set_state(sim.model, sim.data, s0.x_Q, np.zeros(3), np.eye(3), np.zeros(3),
                  _rot([1, 0, 0], swing0) @ np.array([0, 0, -1.0]), np.zeros(3))
    bid = sim.model.body(PAYLOAD_BODY).id
    log, qref, xQref = Logger(), [], []
    held = hover.copy()
    k = 0
    while sim.data.time < duration - 1e-9:
        s = get_state(sim.model, sim.data)
        m = sensor(s)
        r = traj(s.t)
        ctime = 0.0
        if k % ratio == 0:
            tic = time.perf_counter()
            fs = flat_state(r, p)
            cmd = ctrl.load_position(m, r, fs)
            if interface == "CTBR":
                f, Om_cmd = ctbr_from_paper1(ctrl, m)
                cmd = np.array([f, *Om_cmd])
            ctime = time.perf_counter() - tic
            held = delay(cmd)
            qref.append(fs.q); xQref.append(fs.x_Q)
        else:
            qref.append(qref[-1]); xQref.append(xQref[-1])
        if interface == "CTBR":
            wrench = np.array([held[0], *rate(m.Omega, held[1:])])
        else:
            wrench = held
        want = sim.mixer.unconstrained(wrench)
        got = sim.mixer(wrench)
        on = gust_n and gust_window[0] <= s.t < gust_window[1]
        sim.data.xfrc_applied[bid, :3] = [0.0, gust_n, 0.0] if on else 0.0
        log.log(s, tension=cable_tension(sim.model, sim.data, p.g) if log_tension else None, ref=r, wrench=wrench,
                rotor_cmd=got, rotor_unconstrained=want, ctrl_time=ctime if k % ratio == 0 else None)
        sim.step_rotor(got)
        k += 1
        if not np.all(np.isfinite(sim.data.qpos)) or abs(sim.data.qpos[2]) > 1e3:
            break
    L = log.arrays()
    t_end = traj.t_end if traj.t_end is not None else (gust_window[1] if gust_n else None)
    met = compute_metrics(L, sim.control_dt, t_end=t_end, q_ref=np.array(qref), x_Q_ref=np.array(xQref),
                          pendulum_period=2 * np.pi * np.sqrt(p.l / p.g))
    return met, L

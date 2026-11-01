"""Step 2.1 validation checks for Paper 1's flatness map (gates F1-F6)."""
from __future__ import annotations

import sys

import numpy as np

from . import trajectories as T
from .config import load_config
from .flatness import FlatnessSingularity, FlatParams, flat_state, wrench
from .model import drone_info, build_model
from .simulator import Simulator
from .state import get_state, set_state

E3 = np.array([0.0, 0.0, 1.0])
X0 = np.array([0.0, 0.0, 1.5])


def params(cfg: dict | None = None) -> FlatParams:
    cfg = cfg or load_config()
    info = drone_info(build_model(cfg)[0])
    return FlatParams(info.mass, cfg["payload"]["mass"], cfg["cable"]["length"], info.inertia, cfg["sim"]["gravity"])


def test_trajectories() -> dict:
    g = 9.81
    return {"circle_1g": (T.circle(X0, 1.0, g), (1.5, 6.0)),
            "figure8_5s": (T.figure8(X0, 5.0), (2.0, 7.3)),
            "p2p_1s": (T.point_to_point(X0, X0 + [1.0, 0, 0], 1.0), (1.3, 1.7)),
            "ellipse_T2": (T.paper1_ellipse(1.0, 2.0, 1.5, 2.0), (0.4, 1.1))}


def f1_hover() -> dict:
    p = params()
    fs = flat_state(T.hover(X0)(0.0), p)
    errs = {"q": np.abs(fs.q - [0, 0, -1]).max(), "f": abs(fs.f - (p.m_Q + p.m_L) * p.g), "R": np.abs(fs.R - np.eye(3)).max(),
            "Omega": np.abs(fs.Omega).max(), "M": np.abs(fs.M).max(), "T": abs(fs.T - p.m_L * p.g),
            "x_Q": np.abs(fs.x_Q - (X0 + [0, 0, p.l])).max()}
    return {**{f"{k}_err": float(v) for k, v in errs.items()}, "passed": max(errs.values()) < 1e-12}


def f2_equations_of_motion() -> dict:
    """Flat states/inputs satisfy Paper 1's equations of motion (attachment at the CoM):
    (6)  (m_Q + m_L)(a_L + g e3) = (q . f R e3 - m_Q l |q_dot|^2) q
    (8)  m_Q l omega_dot = -q x f R e3,   q_dot = omega x q
    (9)  R_dot = R Omega_hat,   (10) J Omega_dot + Omega x J Omega = M
    """
    p = params()
    worst = 0.0
    for tr, ts in test_trajectories().values():
        for t in ts:
            fs = flat_state(tr(t), p)
            F = fs.f * fs.R[:, 2]
            r6 = (p.m_Q + p.m_L) * (fs.a_L + p.g * E3) - (fs.q @ F - p.m_Q * p.l * fs.q_dot @ fs.q_dot) * fs.q
            r8 = p.m_Q * p.l * fs.omega_dot + np.cross(fs.q, F)
            rq = fs.q_dot - np.cross(fs.omega, fs.q)
            Om_hat = np.array([[0, -fs.Omega[2], fs.Omega[1]], [fs.Omega[2], 0, -fs.Omega[0]],
                               [-fs.Omega[1], fs.Omega[0], 0]])
            r9 = fs.R_jet.d[1] - fs.R @ Om_hat
            r10 = p.J @ fs.Omega_dot + np.cross(fs.Omega, p.J @ fs.Omega) - fs.M
            worst = max(worst, *(float(np.abs(r).max()) for r in (r6, r8, rq, r9, r10)))
    return {"max_residual": worst, "passed": worst < 1e-9}


def _fd_errors(h: float) -> dict:
    p = params()
    out = {"q_dot": 0.0, "v_Q": 0.0, "Omega": 0.0, "Omega_dot": 0.0}
    for tr, ts in test_trajectories().values():
        for t in ts:
            a, b, c = flat_state(tr(t - h), p), flat_state(tr(t), p), flat_state(tr(t + h), p)
            out["q_dot"] = max(out["q_dot"], float(np.abs((c.q - a.q) / (2 * h) - b.q_dot).max()))
            out["v_Q"] = max(out["v_Q"], float(np.abs((c.x_Q - a.x_Q) / (2 * h) - b.v_Q).max()))
            Rdot = (c.R - a.R) / (2 * h)
            S = b.R.T @ Rdot
            Om_fd = np.array([S[2, 1] - S[1, 2], S[0, 2] - S[2, 0], S[1, 0] - S[0, 1]]) / 2
            out["Omega"] = max(out["Omega"], float(np.abs(Om_fd - b.Omega).max()))
            out["Omega_dot"] = max(out["Omega_dot"], float(np.abs((c.Omega - a.Omega) / (2 * h) - b.Omega_dot).max()))
    return out


def f3_consistency() -> dict:
    """Derived rates vs central differences of the flat quantities; convergence ~4x when h halves."""
    e1, e2 = _fd_errors(1e-4), _fd_errors(5e-5)
    out, ok = {}, True
    for k in e1:
        ratio = e1[k] / e2[k]
        out[f"{k}_err"] = e1[k]
        out[f"{k}_ratio"] = ratio
        ok &= 3.5 <= ratio <= 4.5
    return {**out, "passed": ok}


def replay(traj, t0: float, duration: float, cfg: dict | None = None, control_hz: float = 1000.0) -> dict:
    """Open-loop replay: start the simulator exactly in the flat state at t0 and apply only the flatness
    feed-forward wrench (no feedback)."""
    cfg = cfg or load_config()
    sim = Simulator(cfg, control_hz)
    p = params(cfg)
    fs = flat_state(traj(t0), p)
    sim.data.time = t0
    set_state(sim.model, sim.data, fs.x_Q, fs.v_Q, fs.R, fs.Omega, fs.q, fs.q_dot)
    s = get_state(sim.model, sim.data)
    init = max(np.abs(s.x_L - fs.x_L).max(), np.abs(s.v_L - fs.v_L).max(), np.abs(s.x_Q - fs.x_Q).max(),
               np.abs(s.v_Q - fs.v_Q).max(), np.abs(s.R - fs.R).max(), np.abs(s.Omega - fs.Omega).max(),
               np.abs(s.q_dot - fs.q_dot).max())
    t, err = [], []
    while sim.data.time < t0 + duration - 1e-9:
        sim.step_ctbm(wrench(flat_state(traj(sim.data.time), p)))
        s = get_state(sim.model, sim.data)
        t.append(sim.data.time - t0)
        err.append(np.linalg.norm(s.x_L - traj(sim.data.time)[0]))
    t, err = np.array(t), np.array(err)
    over = np.where(err > 0.01)[0]
    return {"init_err": float(init), "err_1s": float(err[np.searchsorted(t, 1.0)]),
            "time_within_1cm": float(t[over[0]]) if len(over) else float(duration), "t": t, "err": err}


def replay_cases() -> dict:
    return {"circle_0.25g": (T.circle(X0, 1.0, 0.25 * 9.81), 4.0),
            "ellipse_T10": (T.paper1_ellipse(1.0, 1.5, 1.5, 10.0), 0.0),
            "p2p_2s": (T.point_to_point(X0, X0 + [1.0, 0, 0], 2.0), 0.0)}


def f4_open_loop_replay(duration: float = 6.0) -> dict:
    out, series, ok = {}, {}, True
    for name, (tr, t0) in replay_cases().items():
        r = replay(tr, t0, duration)
        out[f"{name}_init_err"] = r["init_err"]
        out[f"{name}_err_1s_m"] = r["err_1s"]
        out[f"{name}_within_1cm_s"] = r["time_within_1cm"]
        series[name] = (r["t"], r["err"])
        ok &= r["init_err"] < 1e-12 and r["time_within_1cm"] >= 2.0
    return {**out, "passed": bool(ok), "series": series}


def f5_udaan(udaan_src: str | None) -> dict:
    """Cross-check against Udaan's independent flatness implementation (source path given; not installed)."""
    if not udaan_src:
        return {"skipped": True, "passed": True}
    sys.path.insert(0, udaan_src)
    try:
        from udaan.flatness import Jet as UJet, QuadrotorCsPayload, QuadrotorCsPayloadFlats
    finally:
        sys.path.remove(udaan_src)
    p = params()
    U = QuadrotorCsPayload(mass_q=p.m_Q, mass_l=p.m_L, inertia=p.J, cable_length=p.l, gravity=p.g)
    worst = 0.0
    for tr, ts in test_trajectories().values():
        for t in ts:
            d = tr(t)
            ours = flat_state(d, p)
            ref, inp = U.recover(QuadrotorCsPayloadFlats(x_L=UJet(d), psi=UJet(np.zeros(3))))
            R_ref = np.asarray(getattr(ref.orientation, "matrix", ref.orientation))
            for a, b in ((ours.q, ref.cable_attitude), (ours.omega, ref.cable_angular_velocity),
                         (ours.x_Q, ref.position), (ours.v_Q, ref.velocity), (ours.R, R_ref),
                         (ours.Omega, np.asarray(ref.angular_velocity)), (ours.Omega_dot, ref.angular_acceleration),
                         (ours.f, inp.thrust), (ours.M, inp.moment), (ours.T, inp.tension)):
                worst = max(worst, float(np.max(np.abs(np.asarray(a, float) - np.asarray(b, float)))))
    return {"skipped": False, "max_abs_diff": worst, "passed": worst < 1e-8}


def f6_singularity() -> dict:
    p = params()
    d = T.hover(X0)(0.0)
    d[2] = [0.0, 0.0, -p.g]                      # payload in free fall
    try:
        flat_state(d, p)
        raised = False
    except FlatnessSingularity:
        raised = True
    return {"raises_on_free_fall": raised, "passed": raised}


ALL_CHECKS = {"F1_hover": f1_hover, "F2_equations_of_motion": f2_equations_of_motion,
              "F3_consistency": f3_consistency, "F4_open_loop_replay": f4_open_loop_replay,
              "F6_singularity": f6_singularity}

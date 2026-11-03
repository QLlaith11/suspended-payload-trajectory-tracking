"""Benchmark metrics (D9, step 1.4), computed from a Logger's arrays.

Swing angle = angle between the cable direction q (drone attachment -> payload) and the downward vertical.
"""
from __future__ import annotations

import numpy as np

SETTLE_BAND = 0.05          # m   (D9)
SETTLE_HOLD = 1.0           # s   (D9)
SUCCESS_MAX_ERR = 0.5       # m   (D9)
SUCCESS_MAX_SWING = 60.0    # deg (D9)
CRASH_ALTITUDE = 0.3        # m   (D9, B6)


def swing_angle_deg(q: np.ndarray) -> np.ndarray:
    return np.degrees(np.arccos(np.clip(-q[:, 2], -1.0, 1.0)))


def angle_between_deg(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    cos = np.sum(a * b, axis=1) / (np.linalg.norm(a, axis=1) * np.linalg.norm(b, axis=1))
    return np.degrees(np.arccos(np.clip(cos, -1.0, 1.0)))


def settling_time(t: np.ndarray, err: np.ndarray, t_from: float, band: float = SETTLE_BAND,
                  hold: float = SETTLE_HOLD) -> float:
    """Time after t_from at which the error enters the band for good (stays inside until the end).
    NaN if it never settles or the record does not continue for at least `hold` seconds afterwards."""
    after = t >= t_from - 1e-12
    tt, ee = t[after], err[after]
    outside = np.where(ee > band)[0]
    t_settle = tt[0] if len(outside) == 0 else (tt[outside[-1] + 1] if outside[-1] + 1 < len(tt) else np.nan)
    if np.isnan(t_settle) or tt[-1] - t_settle < hold - 1e-9:
        return float("nan")
    return float(max(0.0, t_settle - t_from))       # max(): removes -0.0 from float round-off


def compute_metrics(log: dict, dt_control: float, t_end: float | None = None, q_ref: np.ndarray | None = None,
                    x_Q_ref: np.ndarray | None = None, pendulum_period: float | None = None) -> dict:
    t = log["t"]
    m: dict = {}

    # Tracking (payload)
    e = log["x_L"] - log["ref"][:, 0, :]
    en = np.linalg.norm(e, axis=1)
    m["payload_rmse"] = float(np.sqrt(np.mean(en**2)))
    for i, ax in enumerate("xyz"):
        m[f"payload_rmse_{ax}"] = float(np.sqrt(np.mean(e[:, i] ** 2)))
    m["payload_max_err"] = float(en.max())
    m["payload_final_err"] = float(en[-1])
    if x_Q_ref is not None:
        m["drone_rmse"] = float(np.sqrt(np.mean(np.sum((log["x_Q"] - x_Q_ref) ** 2, axis=1))))

    # Swing
    sw = swing_angle_deg(log["q"])
    m["swing_max_deg"] = float(sw.max())
    m["swing_rms_deg"] = float(np.sqrt(np.mean(sw**2)))
    if t_end is not None:
        m["swing_residual_deg"] = float(sw[t >= t_end - 1e-12].max())
    if q_ref is not None:
        dyn = angle_between_deg(log["q"], q_ref)
        m["dynamic_swing_max_deg"] = float(dyn.max())
        m["dynamic_swing_rms_deg"] = float(np.sqrt(np.mean(dyn**2)))

    # Transient
    t_from = t_end if t_end is not None else float(t[0])
    ts = settling_time(t, en, t_from)
    m["settling_time_s"] = ts
    if pendulum_period:
        m["settling_time_periods"] = ts / pendulum_period

    # Cable
    if "T" in log:
        # first tick excluded: a soft tendon limit (C2) starts exactly at length L and carries no force for one tick
        Tn = log["T"][1:] if len(log["T"]) > 1 else log["T"]
        m["tension_min_N"] = float(Tn.min())
        m["tension_max_N"] = float(Tn.max())
        m["slack_fraction"] = float(np.mean(Tn <= 0.0))

    # Effort and limits
    if "rotor_cmd" in log:
        m["control_effort"] = float(np.sum(log["rotor_cmd"] ** 2) * dt_control)
    if "saturated" in log:
        m["saturation_fraction"] = float(np.mean(log["saturated"]))
    tilt = np.degrees(np.arccos(np.clip(log["R"][:, 8], -1.0, 1.0)))      # angle between body z and world z
    m["tilt_max_deg"] = float(tilt.max())

    # Success (D9)
    finite = bool(np.all(np.isfinite(log["x_Q"])) and np.all(np.isfinite(log["x_L"])))
    crashed = (not finite) or bool(np.any(log["x_Q"][:, 2] < CRASH_ALTITUDE)) or bool(np.any(tilt > 90.0))
    payload_above = bool(np.any(log["q"][:, 2] > 0.0))
    m["crashed"] = crashed
    m["payload_above_drone"] = payload_above
    m["success"] = bool(not crashed and en.max() < SUCCESS_MAX_ERR and sw.max() < SUCCESS_MAX_SWING
                        and not payload_above)

    # Compute
    if "ctrl_time" in log:
        m["ctrl_time_mean_ms"] = float(np.mean(log["ctrl_time"]) * 1e3)
        m["ctrl_time_max_ms"] = float(np.max(log["ctrl_time"]) * 1e3)
    return m

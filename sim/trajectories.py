"""Payload reference trajectories x_L^d(t) with exact derivatives up to 6th order (D9, step 1.4).

Paper 1's flatness map needs up to the 6th derivative of the payload position (2nd: cable direction,
4th: drone attitude, 5th: body rates, 6th: angular acceleration).

Exactness without slow symbolic expressions:
- polynomial parts (rest-to-rest profile, speed-ramp phase) use exact coefficients (sympy, once) and exact
  polynomial differentiation;
- sin/cos of a phase phi(t) use the exact Leibniz recursion for h = exp(i*phi):
      h^(n) = sum_{k=0}^{n-1} C(n-1, k) * (i*phi)^(k+1) * h^(n-1-k)
  (equivalent to Faa di Bruno's formula), so derivatives are exact up to floating-point round-off.

Evaluation: traj(t) -> array (7, 3), row k = k-th time derivative of the payload position.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from functools import lru_cache
from math import comb

import numpy as np
import sympy as sp

N_DERIV = 6


@lru_cache(maxsize=None)
def smoothstep13_coeffs() -> np.ndarray:
    """Coefficients (highest power first, numpy convention) of the degree-13 polynomial s(u) on [0, 1] with
    s(0)=0, s(1)=1 and derivatives 1..6 zero at both ends."""
    u = sp.symbols("u", real=True)
    c = sp.symbols("c0:14")
    s = sum(ci * u**i for i, ci in enumerate(c))
    eqs = [s.subs(u, 0), s.subs(u, 1) - 1]
    eqs += [sp.diff(s, u, k).subs(u, 0) for k in range(1, N_DERIV + 1)]
    eqs += [sp.diff(s, u, k).subs(u, 1) for k in range(1, N_DERIV + 1)]
    sol = sp.solve(eqs, c)
    return np.array([float(sol[ci]) for ci in reversed(c)])


def _poly_derivs(coeffs: np.ndarray, u: float, n: int) -> np.ndarray:
    """[p(u), p'(u), ..., p^(n)(u)] for a numpy polynomial."""
    out, p = np.empty(n + 1), coeffs
    for k in range(n + 1):
        out[k] = np.polyval(p, u)
        p = np.polyder(p) if len(p) > 1 else np.array([0.0])
    return out


def _exp_i_derivs(phi_d: np.ndarray, scale: float = 1.0) -> np.ndarray:
    """Derivatives 0..N of h = exp(i*scale*phi) given phi_d = [phi, phi', ..., phi^(N)] (exact recursion)."""
    a = 1j * scale * phi_d
    h = np.empty(N_DERIV + 1, complex)
    h[0] = np.exp(a[0])
    for n in range(1, N_DERIV + 1):
        h[n] = sum(comb(n - 1, k) * a[k + 1] * h[n - 1 - k] for k in range(n))
    return h


class SmoothRampPhase:
    """phi(t): rate rises from 0 to omega over t_ramp as omega*s(t/t_ramp) (s = smoothstep13), then constant.
    phi and its derivatives up to the 7th are continuous at t_ramp."""

    def __init__(self, omega: float, t_ramp: float):
        self.omega, self.t_ramp = omega, t_ramp
        self.S = np.polyint(smoothstep13_coeffs())          # integral of s, S(0) = 0, S(1) = 1/2
        self.S1 = float(np.polyval(self.S, 1.0))

    def __call__(self, t: float) -> np.ndarray:
        w, Tr = self.omega, self.t_ramp
        out = np.zeros(N_DERIV + 1)
        if t <= 0.0:
            return out
        if t < Tr:
            d = _poly_derivs(self.S, t / Tr, N_DERIV)
            return w * Tr * d / Tr ** np.arange(N_DERIV + 1)
        out[0] = w * Tr * self.S1 + w * (t - Tr)
        out[1] = w
        return out


@dataclass
class Trajectory:
    name: str
    t_end: float | None = None        # end of the manoeuvre (residual swing / settling); None = periodic or hover
    meta: dict = field(default_factory=dict)

    def __call__(self, t: float) -> np.ndarray:
        raise NotImplementedError


@dataclass
class Hover(Trajectory):
    x0: np.ndarray = None

    def __call__(self, t):
        out = np.zeros((N_DERIV + 1, 3))
        out[0] = self.x0
        return out


@dataclass
class PointToPoint(Trajectory):
    x0: np.ndarray = None
    x1: np.ndarray = None
    duration: float = 1.0
    t_start: float = 1.0

    def __call__(self, t):
        out = np.zeros((N_DERIV + 1, 3))
        u = (t - self.t_start) / self.duration
        if u <= 0:
            out[0] = self.x0
        elif u >= 1:
            out[0] = self.x1
        else:
            d = _poly_derivs(smoothstep13_coeffs(), u, N_DERIV) / self.duration ** np.arange(N_DERIV + 1)
            out = np.outer(d, self.x1 - self.x0)
            out[0] += self.x0
        return out


@dataclass
class PhaseCurve(Trajectory):
    """x = x0 + sum_j amp_j * (sin or 1-cos)(m_j * phi) per axis, with phi from SmoothRampPhase or linear."""
    x0: np.ndarray = None
    terms: list = None            # list of (axis, amplitude, multiplier, kind) with kind in {"sin", "1-cos", "cos"}
    phase: object = None

    def __call__(self, t):
        out = np.zeros((N_DERIV + 1, 3))
        out[0] = self.x0
        phi = self.phase(t)
        for axis, amp, mult, kind in self.terms:
            h = _exp_i_derivs(phi, mult)
            if kind == "sin":
                out[:, axis] += amp * h.imag
            elif kind == "cos":
                out[:, axis] += amp * h.real
            else:  # "1-cos"
                out[:, axis] -= amp * h.real
                out[0, axis] += amp
        return out


class _LinearPhase:
    def __init__(self, omega):
        self.omega = omega

    def __call__(self, t):
        out = np.zeros(N_DERIV + 1)
        out[0], out[1] = self.omega * t, self.omega
        return out


# ----------------------------------------------------------------------------------------------- constructors
PEAK_ACC_FACTOR = 13.176705763387767     # max |s''(u)| of smoothstep13 (step 1.4)


def hover(x0) -> Trajectory:
    return Hover("T0_hover", x0=np.asarray(x0, float))


def point_to_point(x0, x1, duration: float, t_start: float = 1.0) -> Trajectory:
    """T1: rest-to-rest move x0 -> x1 in `duration` s starting at t_start; derivatives 1..6 zero at both ends."""
    x0, x1 = np.asarray(x0, float), np.asarray(x1, float)
    dist = float(np.linalg.norm(x1 - x0))
    return PointToPoint("T1_point_to_point", t_end=t_start + duration,
                        meta={"duration": duration, "distance": dist, "peak_acc": PEAK_ACC_FACTOR * dist / duration**2},
                        x0=x0, x1=x1, duration=duration, t_start=t_start)


def circle(x0, radius: float, peak_acc: float, t_ramp: float = 3.0) -> Trajectory:
    """T2: counter-clockwise horizontal circle through x0 (centre x0 + (0, r, 0)) with steady centripetal
    acceleration `peak_acc` [m/s^2], reached after a smooth speed ramp of t_ramp seconds."""
    omega = float(np.sqrt(peak_acc / radius))
    return PhaseCurve("T2_circle", meta={"radius": radius, "omega": omega, "peak_acc": peak_acc,
                                         "period": 2 * np.pi / omega, "t_ramp": t_ramp},
                      x0=np.asarray(x0, float), terms=[(0, radius, 1.0, "sin"), (1, radius, 1.0, "1-cos")],
                      phase=SmoothRampPhase(omega, t_ramp))


def figure8(x0, period: float, ax: float = 1.0, ay: float = 0.5, t_ramp: float = 3.0) -> Trajectory:
    """T3: Lissajous 1:2 figure-8 through x0 (x amplitude ax, y amplitude ay), period reached after a ramp."""
    omega = 2 * np.pi / period
    return PhaseCurve("T3_figure8", meta={"period": period, "ax": ax, "ay": ay, "t_ramp": t_ramp},
                      x0=np.asarray(x0, float), terms=[(0, ax, 1.0, "sin"), (1, ay, 2.0, "sin")],
                      phase=SmoothRampPhase(omega, t_ramp))


def paper1_ellipse(ax: float, ay: float, az: float, period: float) -> Trajectory:
    """Paper 1 (Sreenath et al. 2013, Eq. 46): x_L = (Ax sin 2πt/T0, Ay cos 2πt/T0, Az), yaw = 0."""
    return PhaseCurve("P1_ellipse", meta={"ax": ax, "ay": ay, "az": az, "period": period},
                      x0=np.array([0.0, 0.0, az]), terms=[(0, ax, 1.0, "sin"), (1, ay, 1.0, "cos")],
                      phase=_LinearPhase(2 * np.pi / period))

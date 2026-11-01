import sys
from math import factorial
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np
import pytest

from sim import checks_flatness
from sim.jet import Jet


def _exp_jet(a: float, t: float, n: int = 6) -> Jet:
    """Jet of exp(a t): k-th derivative a^k exp(a t)."""
    return Jet([a**k * np.exp(a * t) for k in range(n + 1)])


def test_jet_product_reciprocal_sqrt():
    t, n = 0.3, 6
    f, g = _exp_jet(0.7, t), _exp_jet(-1.2, t)
    assert np.allclose((f * g).d, _exp_jet(-0.5, t).d, rtol=1e-12)          # exp(a)exp(b) = exp(a+b)
    assert np.allclose(f.reciprocal().d, _exp_jet(-0.7, t).d, rtol=1e-12)   # 1/exp(a) = exp(-a)
    assert np.allclose((f * f).sqrt().d, f.d, rtol=1e-12)                   # sqrt(exp(a)^2) = exp(a)
    # polynomial t^6: 6th derivative is 6!
    p = Jet([t**6, 6 * t**5, 30 * t**4, 120 * t**3, 360 * t**2, 720 * t, 720])
    assert np.isclose((p * Jet.constant(2.0, n)).d[6], 2 * factorial(6))


def test_jet_vector_normalize_and_cross():
    # unit circle (cos t, sin t, 0) scaled by 3: normalized derivatives are those of the unit circle
    t, n = 0.4, 6
    c = np.array([[np.cos(t + k * np.pi / 2), np.sin(t + k * np.pi / 2), 0.0] for k in range(n + 1)])
    unit, norm = Jet(3.0 * c).normalized()
    assert np.allclose(unit.d, c, atol=1e-12) and np.allclose(norm.d, [3, 0, 0, 0, 0, 0, 0], atol=1e-12)
    z = Jet.constant([0.0, 0.0, 1.0], n)
    assert np.allclose(unit.cross(z).d[0], np.cross(c[0], [0, 0, 1]))


@pytest.mark.parametrize("name", list(checks_flatness.ALL_CHECKS))
def test_flatness_gates(name):
    result = checks_flatness.ALL_CHECKS[name]()
    assert result["passed"], (name, {k: v for k, v in result.items() if k != "series"})

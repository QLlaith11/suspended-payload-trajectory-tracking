import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np
import pytest

from sim import checks_geometric
from sim.checks_flatness import params
from sim.controllers.geometric_paper1 import Gains, GeometricController


def test_default_gains_follow_bandwidths():
    p = params()
    g = Gains.from_bandwidths(p)
    assert g.bandwidths == (1.5, 7.5, 30.0)                       # finding F3
    M = p.m_Q + p.m_L
    assert np.isclose(g.k_x, M * 1.5**2) and np.isclose(g.k_q, p.m_Q * p.l * 7.5**2)
    assert np.allclose(g.K_R, 30.0**2 * p.J)


def test_cable_rate_error_is_tangent_and_zero_on_reference():
    q = np.array([0.0, np.sin(0.3), -np.cos(0.3)])
    q_dot = np.cross([1.0, 0.0, 0.0], q)
    e = GeometricController.cable_rate_error(q, q_dot, q, q_dot)
    assert np.allclose(e, 0.0)                                    # zero when tracking the reference exactly


@pytest.mark.parametrize("name", list(checks_geometric.ALL_CHECKS))
def test_geometric_gates(name):
    result = checks_geometric.ALL_CHECKS[name]()
    assert result["passed"], (name, {k: v for k, v in result.items() if k != "series"})

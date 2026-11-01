import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np
import pytest

from sim import checks_traj
from sim import trajectories as T


@pytest.mark.parametrize("name", list(checks_traj.ALL_CHECKS))
def test_traj_gates(name):
    result = checks_traj.ALL_CHECKS[name]()
    assert result["passed"], (name, result)


def test_shapes_and_start_points():
    x0 = np.array([0.0, 0.0, 1.5])
    for traj in (T.hover(x0), T.point_to_point(x0, x0 + 1, 2.0), T.circle(x0, 1.0, 4.9), T.figure8(x0, 10.0)):
        v = traj(0.0)
        assert v.shape == (7, 3)
        assert np.allclose(v[0], x0) and np.allclose(v[1:], 0.0)   # all start from rest at x0


def test_paper1_ellipse_matches_eq46():
    e = T.paper1_ellipse(1.0, 2.0, 1.5, 4.0)
    t = 0.7
    w = 2 * np.pi / 4.0
    assert np.allclose(e(t)[0], [np.sin(w * t), 2 * np.cos(w * t), 1.5])
    assert np.allclose(e(t)[2], [-w**2 * np.sin(w * t), -2 * w**2 * np.cos(w * t), 0.0])

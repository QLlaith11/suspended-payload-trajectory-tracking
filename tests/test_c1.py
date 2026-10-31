import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np
import pytest

from sim import checks_c1
from sim.config import load_config
from sim.model import build_model
from sim.state import cable_tension, get_state


def test_c1_p1_model_structure():
    model, data = build_model(load_config())
    assert model.nq == 11 and model.nv == 9          # free joint + ball joint
    s = get_state(model, data)
    assert np.allclose(s.q, [0, 0, -1]) and np.isclose(s.cable_length, 0.5)
    assert np.isclose(cable_tension(model, data, 9.81), 0.2 * 9.81)


def test_unimplemented_cable_is_rejected():
    with pytest.raises(NotImplementedError):
        build_model(load_config(overrides={"cable": {"model": "C3"}}))


@pytest.mark.parametrize("name", [n for n in checks_c1.ALL_CHECKS if n != "P2_swing_period"])
def test_c1_gates(name):
    result = checks_c1.ALL_CHECKS[name](load_config())
    assert result["passed"], (name, {k: v for k, v in result.items() if k != "series"})


def test_swing_period_nominal_length():
    measured, theory, _ = checks_c1._period(load_config(), 0.5, duration=8.0)
    assert abs(measured - theory) / theory < 1e-3

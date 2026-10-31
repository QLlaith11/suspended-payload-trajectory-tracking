import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np
import pytest

from sim import checks_actuators
from sim.config import load_config
from sim.simulator import Simulator


@pytest.mark.parametrize("name", list(checks_actuators.ALL_CHECKS))
def test_actuator_gates(name):
    result = checks_actuators.ALL_CHECKS[name](load_config())
    assert result["passed"], (name, {k: v for k, v in result.items() if k != "series"})


def test_simulator_rates():
    cfg = load_config()
    assert Simulator(cfg, 500).substeps == 2
    assert Simulator(cfg, 100).substeps == 10
    with pytest.raises(ValueError):
        Simulator(cfg, 300)  # 1000/300 is not an integer


def test_simulator_hover_ctbm():
    cfg = load_config()
    sim = Simulator(cfg, 500)
    f = sim.info.mass * cfg["sim"]["gravity"]
    for _ in range(500):  # 1 s
        sim.step_ctbm(np.array([f, 0.0, 0.0, 0.0]))
    assert np.linalg.norm(sim.data.qpos[:3] - [0, 0, 2.0]) < 1e-9

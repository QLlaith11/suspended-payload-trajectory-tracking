import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import mujoco
import numpy as np

from sim import checks_x2
from sim.config import load_config
from sim.model import COM_SITE, build_model, drone_info


def test_settings_applied():
    cfg = load_config()
    model, data = build_model(cfg)
    assert model.opt.timestep == 0.001
    assert model.opt.integrator == mujoco.mjtIntegrator.mjINT_RK4
    assert model.opt.disableflags & mujoco.mjtDisableBit.mjDSBL_CONTACT
    assert model.opt.density == 0.0  # L0: no fluid drag
    assert np.allclose(data.qpos[:3], [0, 0, 2.0])
    assert np.allclose(model.site_pos[model.site(COM_SITE).id], drone_info(model).com)


def test_rotor_geometry_cancels_at_hover():
    info = drone_info(build_model(load_config())[0])
    assert np.isclose(info.rotor_pos[:, 0].sum(), 0) and np.isclose(info.rotor_pos[:, 1].sum(), 0)
    assert np.isclose(info.yaw_coeff.sum(), 0)
    assert info.thrust_max == 13.0


def test_gates_pass():
    cfg = load_config()
    for name, check in checks_x2.ALL_CHECKS.items():
        kwargs = {"duration": 1.0} if name == "G2_hover" else {}
        result = check(cfg, **kwargs)
        assert result["passed"], (name, {k: v for k, v in result.items() if k != "series"})

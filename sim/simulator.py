"""Multi-rate simulation loop (D5): physics at 1 kHz, controller commands held constant (zero-order hold)
between controller ticks."""
from __future__ import annotations

import mujoco
import numpy as np

from .actuators import Mixer
from .model import build_model, drone_info, reset


class Simulator:
    def __init__(self, cfg: dict, control_hz: float):
        self.cfg = cfg
        self.model, self.data = build_model(cfg)
        self.info = drone_info(self.model)
        self.mixer = Mixer(self.info)
        steps = self.model.opt.timestep * control_hz
        self.substeps = int(round(1.0 / steps))
        if abs(self.substeps * steps - 1.0) > 1e-9:
            raise ValueError(f"control rate {control_hz} Hz is not an integer divisor of the physics rate")
        self.control_dt = self.substeps * self.model.opt.timestep

    def reset(self) -> None:
        reset(self.model, self.data)

    def step_rotor(self, thrusts: np.ndarray) -> None:
        """Hold the 4 rotor thrust commands for one control period."""
        self.data.ctrl[:] = thrusts
        for _ in range(self.substeps):
            mujoco.mj_step(self.model, self.data)

    def step_ctbm(self, wrench: np.ndarray) -> np.ndarray:
        """Mix a CTBM command (f, tau_x, tau_y, tau_z) and hold it for one control period. Returns rotor commands."""
        thrusts = self.mixer(wrench)
        self.step_rotor(thrusts)
        return thrusts

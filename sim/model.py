"""Build the MuJoCo model from the vendored Skydio X2 (MuJoCo Menagerie) without editing its files.

All modifications are applied in code through MjSpec, each traced to a design decision (SIMULATION_DESIGN.md).
Later steps add the cable (D3) and payload (D6) to the same spec.
"""
from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np

from .config import ROOT

X2_DIR = ROOT / "assets" / "skydio_x2"
X2_SCENE = X2_DIR / "scene.xml"      # x2.xml + floor/skybox (visual only; contacts are disabled)
DRONE_BODY = "x2"
ROTOR_ACTUATORS = ("thrust1", "thrust2", "thrust3", "thrust4")
COM_SITE = "drone_com"


@dataclass
class DroneInfo:
    """Physical properties of the compiled drone, read back from MuJoCo (not hard-coded)."""
    mass: float
    com: np.ndarray            # CoM in the drone body frame [m]
    inertia: np.ndarray        # 3x3 inertia about the CoM, body axes [kg m^2]
    rotor_pos: np.ndarray      # 4x3 rotor (thrust site) positions in the body frame [m]
    yaw_coeff: np.ndarray      # 4 signed yaw-torque/thrust ratios [m]
    thrust_max: float          # [N] per rotor

    def hover_thrust_per_rotor(self, g: float = 9.81) -> float:
        return self.mass * g / 4


def build_spec(cfg: dict) -> mujoco.MjSpec:
    """Load the X2 scene and apply our settings (D2, D5, D9, D10)."""
    spec = mujoco.MjSpec.from_file(str(X2_SCENE))
    opt = spec.option
    opt.timestep = cfg["sim"]["dt"]                                    # D5: 1 ms
    opt.integrator = getattr(mujoco.mjtIntegrator, f"mjINT_{cfg['sim']['integrator']}")  # D5: RK4
    opt.gravity = [0.0, 0.0, -cfg["sim"]["gravity"]]
    if not cfg["sim"]["contacts"]:                                     # D9/B6: no ground contact
        opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_CONTACT
    if not cfg["realism"]["fluid_drag"]:                               # D10: fluid drag off in L0
        opt.density = 0.0
        opt.viscosity = 0.0

    z0 = cfg["sim"]["start_altitude"]                                  # D9/B6: start at 2 m
    spec.body(DRONE_BODY).pos = [0.0, 0.0, z0]
    for key in spec.keys:                                              # keep the "hover" keyframe consistent
        key.qpos = [0.0, 0.0, z0, 1.0, 0.0, 0.0, 0.0]

    thrust_max = cfg["drone"]["rotor_thrust_max"]                      # D2/D8: 0-13 N
    tau_m = cfg["drone"]["motor_tau"]                                  # D8: 0 = ideal motors, 0.035 s benchmark
    for name in ROTOR_ACTUATORS:
        act = spec.actuator(name)
        act.ctrlrange = [0.0, thrust_max]
        if tau_m > 0:
            # First-order motor lag on rotor thrust, integrated by MuJoCo itself (MJX-compatible):
            # tau_m * d(f)/dt = f_cmd - f ; actuator force = activation f.
            # Plain FILTER (not FILTEREXACT): under RK4 it is integrated to 6e-9 N, whereas FILTEREXACT is only
            # exact under Euler/implicit and has 1.6e-2 N error under RK4 (checked in step 1.2).
            act.dyntype = mujoco.mjtDyn.mjDYN_FILTER
            act.dynprm[0] = tau_m

    # Site at the drone CoM (cable attachment reference, D6/A3). The CoM is only known after compiling.
    tmp = spec.compile()
    com = tmp.body_ipos[tmp.body(DRONE_BODY).id].copy()
    spec.body(DRONE_BODY).add_site(name=COM_SITE, pos=com, size=[0.01, 0, 0], rgba=[1, 0, 0, 1], group=4)
    return spec


def build_model(cfg: dict) -> tuple[mujoco.MjModel, mujoco.MjData]:
    model = build_spec(cfg).compile()
    data = mujoco.MjData(model)
    reset(model, data)
    return model, data


def reset(model: mujoco.MjModel, data: mujoco.MjData) -> None:
    """Reset to the 'hover' keyframe: drone at start altitude, level, at rest, hover thrust."""
    mujoco.mj_resetDataKeyframe(model, data, model.key("hover").id)
    if model.na:                       # motor lag on: start the motors in steady state at the commanded thrust
        data.act[:] = data.ctrl[:]
    mujoco.mj_forward(model, data)


def drone_info(model: mujoco.MjModel) -> DroneInfo:
    b = model.body(DRONE_BODY).id
    rot = np.zeros(9)
    mujoco.mju_quat2Mat(rot, model.body_iquat[b])
    rot = rot.reshape(3, 3)
    inertia = rot @ np.diag(model.body_inertia[b]) @ rot.T
    acts = [model.actuator(n) for n in ROTOR_ACTUATORS]
    return DroneInfo(
        mass=float(model.body_mass[b]),      # drone only (payload/cable bodies are added later)
        com=model.body_ipos[b].copy(),
        inertia=inertia,
        rotor_pos=np.array([model.site_pos[a.trnid[0]] for a in acts]),
        yaw_coeff=np.array([a.gear[5] for a in acts]),
        thrust_max=float(acts[0].ctrlrange[1]),
    )

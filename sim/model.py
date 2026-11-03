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
ATTACH_SITE = "cable_attach_drone"
PAYLOAD_BODY = "payload"
PAYLOAD_SITE = "payload_point"
CABLE_JOINT = "cable_ball"
PAYLOAD_FREE_JOINT = "payload_free"     # C2: payload is a free body
CABLE_TENDON = "cable"                  # C2: length-limited tendon


def bare_drone(cfg: dict) -> dict:
    """Same configuration without cable and payload (used by the bare-drone checks of steps 1.1-1.2)."""
    from .config import deep_merge
    return deep_merge(cfg, {"payload": {"model": "none"}})


@dataclass
class DroneInfo:
    """Physical properties of the compiled drone, read back from MuJoCo (not hard-coded)."""
    mass: float
    com: np.ndarray            # CoM in the drone body frame [m]
    inertia: np.ndarray        # 3x3 inertia about the CoM, body axes [kg m^2]
    rotor_pos: np.ndarray      # 4x3 rotor (thrust site) positions in the body frame [m]
    yaw_coeff: np.ndarray      # 4 signed yaw-torque/thrust ratios [m]
    thrust_max: float          # [N] per rotor
    thrust_min: float = 0.0    # [N] per rotor

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

    thrust_max = cfg["drone"]["rotor_thrust_max"]                      # D2/D8: 0-13 N
    tau_m = cfg["drone"]["motor_tau"]                                  # D8: 0 = ideal motors, 0.035 s benchmark
    for name in ROTOR_ACTUATORS:
        act = spec.actuator(name)
        if cfg["drone"].get("ideal_actuator", False):   # Paper 1's assumption: no input limits (theory checks)
            act.ctrlrange = [-1e6, 1e6]
        else:
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
    drone_mass = float(tmp.body_mass[tmp.body(DRONE_BODY).id])
    com = tmp.body_ipos[tmp.body(DRONE_BODY).id].copy()
    drone = spec.body(DRONE_BODY)
    drone.add_site(name=COM_SITE, pos=com, size=[0.01, 0, 0], rgba=[1, 0, 0, 1], group=4)

    payload_mass = 0.0
    if cfg["payload"]["model"] != "none":
        payload_mass = _add_cable_and_payload(spec, cfg, com)

    # Hover keyframe: drone at the start altitude, cable hanging straight down, hover thrust for the total mass.
    hover = (drone_mass + payload_mass) * cfg["sim"]["gravity"] / 4
    extra = []
    if cfg["payload"]["model"] != "none":
        if cfg["cable"]["model"] == "C1":
            extra = [1.0, 0.0, 0.0, 0.0]                          # ball-joint quaternion
        else:                                                     # C2: payload free joint, hanging at length L
            a = com + np.array([0.0, 0.0, cfg["cable"]["attach_offset_z"]])
            extra = [a[0], a[1], z0 + a[2] - cfg["cable"]["length"], 1.0, 0.0, 0.0, 0.0]
    for key in spec.keys:
        key.qpos = [0.0, 0.0, z0, 1.0, 0.0, 0.0, 0.0] + extra
        key.ctrl = [hover] * 4
    return spec


def _add_cable_and_payload(spec: mujoco.MjSpec, cfg: dict, com: np.ndarray) -> float:
    """Add the cable and payload. Step 1.3: C1 (rigid massless link, ball joint) + P1 (point mass).

    C1 (D3): the payload is a child body of the drone, connected by a ball joint at the attachment point, which is
    the drone CoM plus `attach_offset_z` along body z (D6/A3). The ball joint has no singularity at any swing angle.
    P1 (D6): point mass at distance L below the joint. The ball joint also allows twist about the cable, which has
    no physical effect for a point mass; a tiny inertia (1e-6 kg m^2) only keeps MuJoCo's inertia positive.
    """
    cable, payload = cfg["cable"]["model"], cfg["payload"]["model"]
    if cable not in ("C1", "C2") or payload != "P1":
        raise NotImplementedError(f"cable {cable} / payload {payload} not implemented yet (C1/C2 + P1)")
    L, m_L = cfg["cable"]["length"], cfg["payload"]["mass"]
    attach = com + np.array([0.0, 0.0, cfg["cable"]["attach_offset_z"]])

    drone = spec.body(DRONE_BODY)
    drone.add_site(name=ATTACH_SITE, pos=attach, size=[0.008, 0, 0], rgba=[0, 0, 1, 1], group=4)
    if cable == "C2":
        _add_c2(spec, cfg, attach, L, m_L)
        return m_L
    body = drone.add_body(name=PAYLOAD_BODY, pos=attach)
    body.add_joint(name=CABLE_JOINT, type=mujoco.mjtJoint.mjJNT_BALL, damping=0.0)
    body.explicitinertial = True
    body.mass = m_L
    body.ipos = [0.0, 0.0, -L]
    body.inertia = [1e-6, 1e-6, 1e-6]
    body.add_site(name=PAYLOAD_SITE, pos=[0.0, 0.0, -L], size=[0.01, 0, 0], group=4)
    # Visual only (massless, no contacts)
    body.add_geom(type=mujoco.mjtGeom.mjGEOM_CAPSULE, fromto=[0, 0, 0, 0, 0, -L], size=[0.003, 0, 0],
                  mass=0.0, contype=0, conaffinity=0, group=2, rgba=[0.1, 0.1, 0.1, 1])
    body.add_geom(type=mujoco.mjtGeom.mjGEOM_SPHERE, pos=[0.0, 0.0, -L], size=[0.035, 0, 0],
                  mass=0.0, contype=0, conaffinity=0, group=2, rgba=[0.85, 0.35, 0.1, 1])
    return m_L


# C2 tendon-limit softness (D3: stretch < 1 mm while taut). Default solref (0.02 s) is too soft; 4 ms gives 0.01 mm at
# hover and 0.05 mm swinging. Snaps: 2 ms halves the snap stretch but doubles the (non-physical) peak force and resolves
# the snap impulse worse at 1 ms (-35 % vs -20 %) -> 4 ms kept (step 2.6a, finding F6).
C2_SOLREF = (0.004, 1.0)
C2_SOLIMP = (0.99, 0.999, 0.001, 0.5, 2.0)


def _add_c2(spec: mujoco.MjSpec, cfg: dict, attach: np.ndarray, L: float, m_L: float) -> None:
    """C2 (D3): inextensible, slack-capable cable = spatial tendon between the attachment site and the payload with a
    one-sided length limit 0 <= length <= L (as Udaan). Massless; pulls only. P1: payload is a free point mass
    (tiny inertia keeps MuJoCo's inertia positive; rotation has no effect, nothing acts on it)."""
    z0 = cfg["sim"]["start_altitude"]
    body = spec.worldbody.add_body(name=PAYLOAD_BODY, pos=[attach[0], attach[1], z0 + attach[2] - L])
    body.add_freejoint(name=PAYLOAD_FREE_JOINT)
    body.explicitinertial = True
    body.mass = m_L
    body.ipos = [0.0, 0.0, 0.0]
    body.inertia = [1e-6, 1e-6, 1e-6]
    body.add_site(name=PAYLOAD_SITE, pos=[0.0, 0.0, 0.0], size=[0.01, 0, 0], group=4)
    body.add_geom(type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[0.035, 0, 0], mass=0.0, contype=0, conaffinity=0,
                  group=2, rgba=[0.85, 0.35, 0.1, 1])
    t = spec.add_tendon(name=CABLE_TENDON)
    t.limited = mujoco.mjtLimited.mjLIMITED_TRUE
    t.range = [0.0, L]
    t.solref_limit = list(C2_SOLREF)
    t.solimp_limit = list(C2_SOLIMP)
    t.width = 0.003
    t.rgba = [0.1, 0.1, 0.1, 1]
    t.wrap_site(ATTACH_SITE)
    t.wrap_site(PAYLOAD_SITE)


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
        thrust_min=float(acts[0].ctrlrange[0]),
    )

"""Open the MuJoCo viewer with the X2 hovering at exact hover thrust (open loop). Close the window to quit."""
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import mujoco
import mujoco.viewer

from sim.config import load_config
from sim.model import build_model, drone_info


def main():
    cfg = load_config()
    model, data = build_model(cfg)
    f = drone_info(model).hover_thrust_per_rotor(cfg["sim"]["gravity"])
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            start = time.time()
            data.ctrl[:] = f
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(max(0.0, model.opt.timestep - (time.time() - start)))


if __name__ == "__main__":
    main()

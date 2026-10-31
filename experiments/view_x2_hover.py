"""Open the MuJoCo viewer: X2 with the C1 cable and P1 payload, open loop at hover thrust.

The payload is released with a 20° swing so the pendulum motion is visible (no controller yet: the drone
translates as the payload swings). Use --bare for the drone alone. Close the window to quit.
"""
import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import mujoco
import mujoco.viewer
import numpy as np

from sim.config import load_config
from sim.model import bare_drone, build_model


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bare", action="store_true", help="drone without cable/payload")
    parser.add_argument("--swing", type=float, default=20.0, help="initial payload swing [deg]")
    args = parser.parse_args()

    cfg = load_config()
    if args.bare:
        cfg = bare_drone(cfg)
    model, data = build_model(cfg)
    hover = data.ctrl.copy()
    if not args.bare:
        a = np.radians(args.swing)
        data.qpos[7:11] = [np.cos(a / 2), 0.0, np.sin(a / 2), 0.0]
        mujoco.mj_forward(model, data)
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            start = time.time()
            data.ctrl[:] = hover
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(max(0.0, model.opt.timestep - (time.time() - start)))


if __name__ == "__main__":
    main()

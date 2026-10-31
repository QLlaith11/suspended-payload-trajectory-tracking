"""Step 1.2 gate: validate the actuator stack (A1-A6). See IMPLEMENTATION_PLAN.md."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from sim import checks_actuators
from sim.actuators import Mixer
from sim.config import load_config
from sim.model import build_model, drone_info
from sim.run_io import RunRecorder


def main():
    cfg = load_config()
    run = RunRecorder("phase1", "e12_actuator_validation", cfg,
                      description="Step 1.2: CTBM -> mixer (PX4-style priorities) -> 0-13 N -> motor lag "
                                  "(MuJoCo FILTER actuator); gates A1-A6.")
    results = {name: check(cfg) for name, check in checks_actuators.ALL_CHECKS.items()}
    metrics = {f"{n}.{k}": v for n, r in results.items() for k, v in r.items() if k != "series"}
    metrics["all_passed"] = all(r["passed"] for r in results.values())
    run.save_metrics(metrics)

    s5 = results["A5_motor_lag"]["series"]
    run.save_data(**{f"A5_{k}": v for k, v in s5.items()})

    # Saturation illustration: thrust ramp with a fixed roll + yaw request
    mixer = Mixer(drone_info(build_model(cfg)[0]))
    f_req = np.linspace(0, 60, 300)
    ach = np.array([mixer.wrench(mixer([f, 0.4, 0.0, 0.3])) for f in f_req])

    fig, ax = plt.subplots(1, 2, figsize=(11, 3.4))
    ax[0].plot(s5["t"] * 1e3, s5["f_lag"], label="MuJoCo (τ = 35 ms)")
    ax[0].plot(s5["t"] * 1e3, s5["f_expected"], "--", label="6 − 3·exp(−t/τ)")
    ax[0].plot(s5["t"] * 1e3, s5["f_ideal"], ":", label="ideal motor")
    ax[0].axvline(35, color="gray", lw=0.8)
    ax[0].set(title="A5 motor step response", xlabel="t [ms]", ylabel="rotor thrust [N]")
    ax[0].legend()
    ax[1].plot(f_req, ach[:, 0], label="achieved thrust [N]")
    ax[1].plot(f_req, ach[:, 1] * 100, label="achieved roll torque ×100 [N m]")
    ax[1].plot(f_req, ach[:, 3] * 100, label="achieved yaw torque ×100 [N m]")
    ax[1].set(title="A4 saturation priorities (request: τx = 0.4, τz = 0.3)", xlabel="requested thrust [N]")
    ax[1].legend(fontsize=8)
    run.save_figure(fig, "actuators")

    out = run.finish(notes="Gate passes only if all_passed is true. Motor lag uses MuJoCo dyntype FILTER "
                           "(FILTEREXACT is inaccurate under RK4).")
    for name, res in results.items():
        print(f"{name}: {'PASS' if res['passed'] else 'FAIL'}  " +
              ", ".join(f"{k}={v:.3g}" if isinstance(v, float) else f"{k}={v}"
                        for k, v in res.items() if k not in ("series", "passed")))
    print(out)


if __name__ == "__main__":
    main()

"""Step 1.4 gate: trajectories (exact derivatives to 6th order), metrics, logger (M1-M5)."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from sim import checks_traj
from sim import trajectories as T
from sim.config import load_config
from sim.run_io import RunRecorder


def main():
    cfg = load_config()
    run = RunRecorder("phase1", "e14_trajectories_metrics", cfg,
                      description="Step 1.4: payload reference trajectories T0-T3 + Paper 1 ellipse with exact "
                                  "derivatives up to 6th order; D9 metrics; logger. Gates M1-M5.")
    results = {name: check() for name, check in checks_traj.ALL_CHECKS.items()}
    metrics = {f"{n}.{k}": v for n, r in results.items() for k, v in r.items()}
    metrics["all_passed"] = all(r["passed"] for r in results.values())
    run.save_metrics(metrics)

    x0, g = np.array([0.0, 0.0, 1.5]), 9.81
    trajs = {
        "T1 1 m in 1 s": (T.point_to_point(x0, x0 + [1, 0, 0], 1.0), 4.0),
        "T2 circle 0.5 g": (T.circle(x0, 1.0, 0.5 * g), 10.0),
        "T3 figure-8 10 s": (T.figure8(x0, 10.0), 16.0),
        "Paper 1 ellipse T0 = 4 s": (T.paper1_ellipse(1.0, 2.0, 1.5, 4.0), 4.0),
    }
    fig, ax = plt.subplots(2, 4, figsize=(16, 6.5))
    data = {}
    for j, (label, (tr, dur)) in enumerate(trajs.items()):
        ts = np.linspace(0, dur, 1500)
        v = np.array([tr(t) for t in ts])
        data[f"traj{j}_t"], data[f"traj{j}_derivs"] = ts, v
        ax[0, j].plot(v[:, 0, 0], v[:, 0, 1])
        ax[0, j].set(title=label, xlabel="x [m]", ylabel="y [m]")
        ax[0, j].axis("equal")
        for k, name in ((1, "vel"), (2, "acc"), (3, "jerk"), (4, "snap")):
            mag = np.linalg.norm(v[:, k, :], axis=1)
            ax[1, j].semilogy(ts, np.maximum(mag, 1e-6), label=name)
        ax[1, j].set(xlabel="t [s]", ylabel="norm of derivative")
        ax[1, j].legend(fontsize=7)
    run.save_data(**data)
    run.save_figure(fig, "trajectories")

    out = run.finish(notes="Gate passes only if all_passed is true. M1 compares against an independent symbolic "
                           "(sympy) differentiation; M2 is a convergence test for smoothness at the ramp join.")
    for name, res in results.items():
        print(f"{name}: {'PASS' if res['passed'] else 'FAIL'}  " +
              ", ".join(f"{k}={v:.3g}" if isinstance(v, float) else f"{k}={v}"
                        for k, v in res.items() if k != "passed"))
    print(out)


if __name__ == "__main__":
    main()

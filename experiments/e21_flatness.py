"""Step 2.1 gate: Paper 1 flatness map (Sreenath, Lee, Kumar, CDC 2013, Sec. III), gates F1-F6.

Usage: python3 experiments/e21_flatness.py [--udaan-src PATH]
  --udaan-src: path to a downloaded (not installed) Udaan source tree for the F5 cross-check.
"""
import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from sim import checks_flatness as C
from sim import trajectories as T
from sim.config import load_config
from sim.flatness import flat_state
from sim.run_io import RunRecorder


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--udaan-src", default=None)
    args = parser.parse_args()

    cfg = load_config()
    run = RunRecorder("phase2", "e21_flatness", cfg,
                      description="Step 2.1: Paper 1 flatness map x_L, psi -> q, T, x_Q, R, Omega, Omega_dot, f, M "
                                  "with exact derivatives (jet arithmetic). Gates F1-F6.")
    results = {name: check() for name, check in C.ALL_CHECKS.items()}
    results["F5_udaan"] = C.f5_udaan(args.udaan_src)
    if args.udaan_src:
        results["F5_udaan"]["udaan_commit"] = "8a3914d62359f3a9750e6081251be64c48e51660"
    metrics = {f"{n}.{k}": v for n, r in results.items() for k, v in r.items() if k != "series"}
    metrics["all_passed"] = all(r["passed"] for r in results.values())
    run.save_metrics(metrics)

    fig, ax = plt.subplots(1, 2, figsize=(12, 4))
    data = {}
    for name, (t, err) in results["F4_open_loop_replay"]["series"].items():
        ax[0].semilogy(t, np.maximum(err, 1e-16), label=name)
        data[f"{name}_t"], data[f"{name}_err"] = t, err
    ax[0].axhline(0.01, color="gray", ls="--", lw=0.8)
    ax[0].set(title="F4 open-loop replay (feed-forward only, no feedback)", xlabel="t [s]",
              ylabel="payload position error [m]")
    ax[0].legend(fontsize=8)

    p = C.params(cfg)
    tr = T.paper1_ellipse(1.0, 2.0, 1.5, 4.0)
    ts = np.linspace(0, 4.0, 400)
    fs = [flat_state(tr(t), p) for t in ts]
    xL, xQ = np.array([f.x_L for f in fs]), np.array([f.x_Q for f in fs])
    ax[1].plot(xL[:, 0], xL[:, 1], label="payload (flat output)")
    ax[1].plot(xQ[:, 0], xQ[:, 1], "--", label="drone (from flatness)")
    for k in range(0, 400, 25):
        ax[1].plot([xQ[k, 0], xL[k, 0]], [xQ[k, 1], xL[k, 1]], color="gray", lw=0.6)
    ax[1].set(title="Paper 1 ellipse, T0 = 4 s: drone path required", xlabel="x [m]", ylabel="y [m]")
    ax[1].axis("equal")
    ax[1].legend(fontsize=8)
    run.save_data(**data)
    run.save_figure(fig, "flatness")

    out = run.finish(notes="Gate passes only if all_passed is true. F5 compares with Udaan (BSD-3, commit 8a3914d) "
                           "loaded from source in a temporary folder, not installed and not copied into the project.")
    for name, res in results.items():
        print(f"{name}: {'PASS' if res['passed'] else 'FAIL'}  " +
              ", ".join(f"{k}={v:.3g}" if isinstance(v, float) else f"{k}={v}"
                        for k, v in res.items() if k not in ("series", "passed")))
    print(out)


if __name__ == "__main__":
    main()

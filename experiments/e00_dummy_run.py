"""Step 0.1 gate: a dummy run that exercises the full run-recording pipeline (no physics)."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from sim.config import load_config
from sim.run_io import RunRecorder


def main():
    cfg = load_config()
    rng = np.random.default_rng(cfg["seed"])
    t = np.arange(0.0, 2.0, cfg["sim"]["dt"])
    signal = np.sin(2 * np.pi * t) + 0.01 * rng.standard_normal(t.size)

    run = RunRecorder("phase0", "e00_dummy_run", cfg,
                      description="Pipeline check: synthetic sine wave, no physics.")
    run.save_data(t=t, signal=signal)
    run.save_metrics({"n_samples": int(t.size), "rms": round(float(np.sqrt(np.mean(signal**2))), 6)})
    fig, ax = plt.subplots(figsize=(6, 3))
    ax.plot(t, signal)
    ax.set_xlabel("t [s]")
    ax.set_ylabel("signal")
    run.save_figure(fig, "signal")
    out = run.finish()
    print(out)


if __name__ == "__main__":
    main()

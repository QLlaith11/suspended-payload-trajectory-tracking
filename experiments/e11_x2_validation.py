"""Step 1.1 gate: validate the bare Skydio X2 model (G1-G5). See IMPLEMENTATION_PLAN.md."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from sim import checks_x2
from sim.config import load_config
from sim.run_io import RunRecorder


def main():
    cfg = load_config()
    run = RunRecorder("phase1", "e11_x2_validation", cfg,
                      description="Step 1.1: Skydio X2 (MuJoCo Menagerie) loaded with our settings; gates G1-G5.")
    results = {name: check(cfg) for name, check in checks_x2.ALL_CHECKS.items()}

    metrics = {}
    for name, res in results.items():
        for k, v in res.items():
            if k != "series":
                metrics[f"{name}.{k}"] = v if not isinstance(v, float) else float(f"{v:.6g}")
    metrics["all_passed"] = all(r["passed"] for r in results.values())
    run.save_metrics(metrics)

    s2, s3, s4 = results["G2_hover"]["series"], results["G3_free_fall"]["series"], results["G4_drag"]["series"]
    run.save_data(**{f"G2_{k}": v for k, v in s2.items()}, **{f"G3_{k}": v for k, v in s3.items()},
                  **{f"G4_{k}": v for k, v in s4.items()})

    fig, ax = plt.subplots(1, 3, figsize=(13, 3.2))
    ax[0].plot(s2["t"], s2["drift"] * 1e3)
    ax[0].set(title="G2 hover: position drift", xlabel="t [s]", ylabel="drift [mm]")
    ax[1].plot(s3["t"], s3["err"] * 1e9)
    ax[1].set(title="G3 free fall: error vs ½gt²", xlabel="t [s]", ylabel="error [nm]")
    ax[2].plot(s4["t"], s4["vz_off"], label="drag off")
    ax[2].plot(s4["t"], s4["vz_on"], "--", label="drag on")
    ax[2].set(title="G4 free fall: vertical speed", xlabel="t [s]", ylabel="v_z [m/s]")
    ax[2].legend()
    run.save_figure(fig, "gates")

    out = run.finish(notes="Gate passes only if all_passed is true.")
    for name, res in results.items():
        print(f"{name}: {'PASS' if res['passed'] else 'FAIL'}  " +
              ", ".join(f"{k}={v:.4g}" if isinstance(v, float) else f"{k}={v}"
                        for k, v in res.items() if k not in ("series", "passed")))
    print(out)


if __name__ == "__main__":
    main()

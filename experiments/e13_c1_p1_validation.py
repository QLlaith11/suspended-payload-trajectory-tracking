"""Step 1.3 gate: cable C1 (rigid massless link) + payload P1 (point mass), gates P1-P7.
Includes experiment X2 (natural swing dynamics vs theory) of SIMULATION_DESIGN.md D4 for C1."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from sim import checks_c1
from sim.config import load_config
from sim.run_io import RunRecorder


def main():
    cfg = load_config()
    run = RunRecorder("phase1", "e13_c1_p1_validation", cfg,
                      description="Step 1.3: C1 rigid massless cable (ball joint at drone CoM) + P1 point-mass "
                                  "payload, nominal L = 0.5 m, m_L = 0.2 kg; gates P1-P7.")
    results = {name: check(cfg) for name, check in checks_c1.ALL_CHECKS.items()}
    metrics = {f"{n}.{k}": v for n, r in results.items() for k, v in r.items() if k != "series"}
    metrics["all_passed"] = all(r["passed"] for r in results.values())
    run.save_metrics(metrics)

    series = results["P2_swing_period"]["series"]
    run.save_data(**{f"{k}_t": v[0] for k, v in series.items()}, **{f"{k}_qx": v[1] for k, v in series.items()})
    fig, ax = plt.subplots(figsize=(8, 3.4))
    for k, (t, qx) in series.items():
        L = float(k[1:])
        T_th = results["P2_swing_period"][f"{k}_theory_s"]
        ax.plot(t, np.degrees(np.arcsin(qx)), label=f"L = {L} m (period {T_th:.3f} s theory)")
    ax.set(title="P2 / X2: free swing at hover thrust (2° release)", xlabel="t [s]", ylabel="swing angle in x [deg]")
    ax.legend(fontsize=8)
    run.save_figure(fig, "swing_period")

    out = run.finish(notes="Gate passes only if all_passed is true. P6 is a convergence test "
                           "(errors vs finite differences must shrink ~4x when dt halves).")
    for name, res in results.items():
        print(f"{name}: {'PASS' if res['passed'] else 'FAIL'}  " +
              ", ".join(f"{k}={v:.4g}" if isinstance(v, float) else f"{k}={v}"
                        for k, v in res.items() if k not in ("series", "passed")))
    print(out)


if __name__ == "__main__":
    main()

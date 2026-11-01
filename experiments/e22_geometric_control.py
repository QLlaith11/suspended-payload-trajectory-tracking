"""Step 2.2 gate: Paper 1 geometric controller, three modes (gates K1-K7). See IMPLEMENTATION_PLAN.md and
finding F3 in SIMULATION_DESIGN.md (gains 1.5 / 7.5 / 30 rad/s; ideal actuator only for theory gates K1, K2, K5)."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from sim import checks_geometric as C
from sim.config import load_config
from sim.run_io import RunRecorder


def main():
    cfg = load_config()
    run = RunRecorder("phase2", "e22_geometric_control", cfg,
                      description="Step 2.2: Paper 1 geometric controller (quadrotor attitude, load attitude, load "
                                  "position), C1+P1, L0, CTBM at 500 Hz, gains 1.5/7.5/30 rad/s. Gates K1-K7.")
    results = {name: check() for name, check in C.ALL_CHECKS.items()}
    metrics = {f"{n}.{k}": v for n, r in results.items() for k, v in r.items() if k != "series"}
    metrics["all_passed"] = all(r["passed"] for r in results.values())
    run.save_metrics(metrics)

    fig, ax = plt.subplots(1, 4, figsize=(20, 4.2))
    data = {}
    t, e = results["K1_attitude_mode"]["series"]
    ax[0].semilogy(t, np.maximum(e, 1e-16))
    ax[0].set(title="K1 (a): |e_R|, 60° start", xlabel="t [s]")
    data["K1_t"], data["K1_eR"] = t, e
    t, psi = results["K2_load_attitude_mode"]["series"]
    ax[1].semilogy(t, np.maximum(psi, 1e-16))
    ax[1].set(title="K2 (b): Ψ_q, 120° start", xlabel="t [s]")
    data["K2_t"], data["K2_psi"] = t, psi
    t, e = results["K3_hover_recovery"]["series"]
    ax[2].plot(t, e)
    ax[2].axhline(0.05, color="gray", ls="--", lw=0.8)
    ax[2].set(title=f"K3 (c): 1 m step, settles {results['K3_hover_recovery']['settling_time_s']:.2f} s "
                    f"(theory {results['K3_hover_recovery']['expected_s']:.2f})", xlabel="t [s]", ylabel="|e_x| [m]")
    data["K3_t"], data["K3_err"] = t, e
    for name, (t, e) in results["K4_tracking"]["series"].items():
        ax[3].semilogy(t, np.maximum(e, 1e-12), label=name)
        data[f"K4_{name}_t"], data[f"K4_{name}_err"] = t, e
    ax[3].set(title="K4 (c): tracking, flatness feed-forward", xlabel="t [s]", ylabel="|e_x| [m]")
    ax[3].legend(fontsize=8)
    run.save_data(**data)
    fig.tight_layout()
    run.save_figure(fig, "geometric_control")

    out = run.finish(notes="Gate passes only if all_passed is true. K1, K2, K5 run with the ideal actuator (Paper 1's "
                           "unlimited-input assumption); all others with the X2's 0-13 N rotor limits (finding F3).")
    for name, res in results.items():
        print(f"{name}: {'PASS' if res['passed'] else 'FAIL'}  " +
              ", ".join(f"{k}={v:.3g}" if isinstance(v, float) else f"{k}={v}"
                        for k, v in res.items() if k not in ("series", "passed")))
    print(out)


if __name__ == "__main__":
    main()

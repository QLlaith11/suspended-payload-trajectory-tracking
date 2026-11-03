"""Step 2.4 (E1.2): Paper 1 recovery from a near-flipped start (Figs. 4-5). Experiment 1: 178 deg, ideal actuator
(gated). Experiment 2: X2 rotor limits, flip-angle sweep (reported). See sim/study_recovery.py."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from sim import study_recovery as R
from sim.config import load_config
from sim.run_io import RunRecorder

KEYS = ("recovered", "t_recover_s", "final_err_m", "height_loss_m", "neg_tension_frac", "min_tension_N", "sat_frac",
        "diverged")


def main():
    run = RunRecorder("phase2", "e24_recovery", load_config(),
                      description="Step 2.4 E1.2: drone + cable flipped by theta about y, load 2 m from reference, "
                                  "reference ellipse 0.5x1.0 m T0=5 s, 10 s. Exp 1: 178 deg ideal actuator (Paper 1). "
                                  "Exp 2: X2 limits, theta sweep.")
    E1 = R.experiment_1()
    E2 = R.experiment_2()
    m, data = {}, {}
    for k in KEYS:
        m[f"E1.{k}"] = E1[k]
    m["E1.Psi_R_final"] = float(E1["series"]["Psi_R"][-1])
    m["E1.Psi_q_final"] = float(E1["series"]["Psi_q"][-1])
    m["E1.passed"] = E1["passed"]
    for k, v in E1["series"].items():
        data[f"E1_{k}"] = v
    for a, r in E2["runs"].items():
        for k in KEYS:
            m[f"E2.theta{a:g}.{k}"] = r[k]
        data[f"E2_theta{a:g}_t"] = r["series"]["t"]
        data[f"E2_theta{a:g}_e_x"] = r["series"]["e_x"]
        data[f"E2_theta{a:g}_z_Q"] = r["series"]["z_Q"]
    m["E2.recovered_angles"] = [a for a, r in E2["runs"].items() if r["recovered"]]
    m["E2.passed"] = E2["passed"]
    m["all_passed"] = bool(E1["passed"] and E2["passed"])
    run.save_metrics(m)
    run.save_data(**data)

    # Fig. 1: Paper 1 Fig. 5 analogue (ideal actuator, 178 deg)
    S = E1["series"]
    fig, ax = plt.subplots(1, 4, figsize=(20, 4.2))
    ax[0].semilogy(S["t"], np.maximum(S["Psi_R"], 1e-16)); ax[0].set(title="Ψ_R (drone attitude error)", xlabel="t [s]")
    ax[1].semilogy(S["t"], np.maximum(S["Psi_q"], 1e-16)); ax[1].set(title="Ψ_q (cable attitude error)", xlabel="t [s]")
    ax[2].semilogy(S["t"], S["e_x"]); ax[2].set(title="|e_x| load position error [m]", xlabel="t [s]")
    ax[3].plot(S["t"], S["tension"]); ax[3].axhline(0, color="r", lw=0.8)
    ax[3].set(title=f"cable tension [N] (<0 = rod pushes: {100 * E1['neg_tension_frac']:.1f} % of time)", xlabel="t [s]")
    fig.suptitle("Exp 1: 178° flip, ideal actuator (Paper 1 Figs. 4-5)")
    fig.tight_layout()
    run.save_figure(fig, "E1_paper1_recovery")

    # Fig. 2: X2 sweep
    angs = np.array(list(E2["runs"]))
    rec = np.array([E2["runs"][a]["recovered"] for a in angs])
    fig, ax = plt.subplots(1, 3, figsize=(18, 4.5))
    ax[0].scatter(angs[rec], [E2["runs"][a]["t_recover_s"] for a in angs[rec]], c="g", label="recovered")
    ax[0].scatter(angs[~rec], np.full((~rec).sum(), 10.5), c="r", marker="x", label="not recovered in 10 s")
    ax[0].set(title="X2: time to enter and hold the 5 cm band", xlabel="initial flip θ [deg]", ylabel="[s]")
    ax[0].legend(fontsize=8)
    ax[1].semilogy(angs, [max(E2["runs"][a]["height_loss_m"], 1e-3) for a in angs], "o-")
    ax[1].set(title="X2: height lost [m]", xlabel="θ [deg]")
    ax[2].plot(angs, [E2["runs"][a]["sat_frac"] for a in angs], "o-", label="rotor saturation")
    ax[2].plot(angs, [E2["runs"][a]["neg_tension_frac"] for a in angs], "s--", label="negative tension")
    ax[2].set(title="X2: fraction of time", xlabel="θ [deg]")
    ax[2].legend(fontsize=8)
    fig.tight_layout()
    run.save_figure(fig, "E2_x2_sweep")

    out = run.finish(notes="all_passed gates Exp 1 (Paper 1 reproduction) and that Exp 2 runs finish numerically; Exp 2 "
                           "outcomes are reported, not gated.")
    print("E1", E1["passed"], {k: E1[k] for k in KEYS})
    for a, r in E2["runs"].items():
        print(f"E2 theta={a:g}: rec {r['recovered']} t {r['t_recover_s']:.2f} final {r['final_err_m']:.3g} "
              f"drop {r['height_loss_m']:.1f} sat {r['sat_frac']:.2f} negT {r['neg_tension_frac']:.3f}")
    print("ALL", m["all_passed"])
    print(out)


if __name__ == "__main__":
    main()

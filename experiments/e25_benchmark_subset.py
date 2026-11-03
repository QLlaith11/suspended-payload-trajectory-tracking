"""Step 2.5 (E1.3): benchmark subset T0-T3 (D9) for Paper 1's controller, nominal settings, X2 and ideal actuator.
See sim/benchmark.py."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from sim import benchmark as B
from sim.config import load_config
from sim.metrics import swing_angle_deg
from sim.run_io import RunRecorder


def main():
    run = RunRecorder("phase2", "e25_benchmark_subset", load_config(),
                      description="Step 2.5 E1.3: D9 benchmark T0 (swing 15/30 deg, gust 2 N 0.5 s), T1 (1 m in 4/2/1 s), "
                                  "T2 (r=1 m circle 0.25/0.5/1 g), T3 (figure-8 15/10/5 s); Paper 1 controller, X2 and "
                                  "ideal actuator; L0 deterministic, one run per case.")
    res = B.run_all()
    m, data = {}, {}
    for name, v in res["cases"].items():
        for k, val in v["feasibility"].items():
            m[f"{name}.{k}"] = val
        for act in ("x2", "ideal"):
            for k, val in v[act].items():
                m[f"{name}.{act}.{k}"] = val
            L = v[f"log_{act}"]
            for k in ("t", "x_L", "x_Q", "q", "T", "saturated"):
                data[f"{name}_{act}_{k}"] = L[k]
            data[f"{name}_{act}_ref"] = L["ref"][:, 0, :]
    m["all_passed"] = res["passed"]
    run.save_metrics(m)
    run.save_data(**data)
    md, csv = B.table(res)
    (run.dir / "results_table.md").write_text(md + "\n")
    (run.dir / "results_table.csv").write_text(csv + "\n")

    names = list(res["cases"])
    nr = int(np.ceil(len(names) / 4))
    fig, ax = plt.subplots(nr, 4, figsize=(20, 3.7 * nr))
    for a in ax.flat[len(names):]:
        a.axis("off")
    for a, name in zip(ax.flat, names):
        for act, st in (("x2", "-"), ("ideal", "--")):
            L = res["cases"][name][f"log_{act}"]
            e = np.linalg.norm(L["x_L"] - L["ref"][:, 0, :], axis=1)
            a.semilogy(L["t"], np.maximum(e, 1e-9), st, label=f"|e_x| {act}")
        L = res["cases"][name]["log_x2"]
        a2 = a.twinx()
        a2.plot(L["t"], swing_angle_deg(L["q"]), color="gray", lw=0.7)
        a2.set_ylabel("swing [°] (X2)", color="gray")
        feas = res["cases"][name]["feasibility"]["feasible"]
        a.set_title(f"{name}{'' if feas else '  (NOT flat-feasible on X2)'}", fontsize=10)
        a.set_xlabel("t [s]")
        a.legend(fontsize=7, loc="lower right")
    fig.tight_layout()
    run.save_figure(fig, "benchmark_T0_T3")
    out = run.finish(notes="Reported, not gated; all_passed = every run finished numerically. Tables: results_table.md/.csv.")
    print(md)
    print("ALL", res["passed"])
    print(out)


if __name__ == "__main__":
    main()

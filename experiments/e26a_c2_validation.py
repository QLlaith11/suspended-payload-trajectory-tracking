"""Step 2.6a gate: cable C2 (inextensible, slack-capable tendon) + P1, gates S1-S6. See sim/checks_c2.py, finding F6."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from sim import checks_c2 as C
from sim.config import load_config
from sim.run_io import RunRecorder


def main():
    cfg = load_config(overrides={"cable": {"model": "C2"}})
    run = RunRecorder("phase2", "e26a_c2_validation", cfg,
                      description="Step 2.6a: C2 tendon cable (0 <= length <= L, solref 4 ms) + P1, drone at hover thrust. "
                                  "Gates S1-S6.")
    res = {n: f() for n, f in C.ALL_CHECKS.items()}
    m = {f"{n}.{k}": v for n, r in res.items() for k, v in r.items() if k != "series"}
    m["all_passed"] = all(r["passed"] for r in res.values())
    run.save_metrics(m)
    t, length, T = res["S4_slack_and_retaut"]["series"]
    run.save_data(S4_t=t, S4_length=length, S4_tension=T)
    fig, ax = plt.subplots(1, 2, figsize=(13, 4))
    ax[0].plot(t, length); ax[0].axhline(cfg["cable"]["length"], color="r", lw=0.8)
    ax[0].set(title="S4: cable length, payload thrown up at 3 m/s", xlabel="t [s]", ylabel="[m]")
    ax[1].plot(t, T); ax[1].set(title="S4: tension (peak NOT physical for C2, F6)", xlabel="t [s]", ylabel="[N]")
    fig.tight_layout()
    run.save_figure(fig, "c2_slack_retaut")
    print(run.finish(notes="Gate passes only if all_passed is true."))
    for n, r in res.items():
        print(n, "PASS" if r["passed"] else "FAIL")


if __name__ == "__main__":
    main()

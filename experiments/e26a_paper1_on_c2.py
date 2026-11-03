"""Step 2.6a (E1.4a): Paper 1's controller on the slack-capable cable C2 (dt 0.5 ms, D5 exception / F6).
Repeats step 2.4 (recovery: 178 deg ideal actuator + X2 flip sweep) and step 2.5 (benchmark T0-T3) with C2 instead of
the rigid link C1, and adds D9's T5(a) slack test (payload thrown upward at 3 m/s from hover, controller on).
T5(b) (commanded descent faster than g) is not run: its reference makes the payload tension negative, so Paper 1's
flatness map is undefined (FlatnessSingularity) - Paper 1 cannot even form the command.
Reported, not gated: all_passed = every run finished numerically."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import mujoco
import numpy as np

from sim import benchmark as B
from sim import study_recovery as R
from sim import trajectories as T
from sim.checks_flatness import params
from sim.config import deep_merge, load_config
from sim.controllers.geometric_paper1 import Gains, GeometricController
from sim.flatness import flat_state
from sim.logger import Logger
from sim.metrics import compute_metrics
from sim.run_io import RunRecorder
from sim.simulator import Simulator
from sim.state import cable_tension, get_state

C2 = {"cable": {"model": "C2"}, "sim": {"dt": 0.0005}}
KEYS = ("recovered", "t_recover_s", "final_err_m", "height_loss_m", "neg_tension_frac", "min_tension_N", "sat_frac")


def t5_throw(ideal: bool, v_up: float = 3.0, duration: float = 6.0):
    cfg = load_config(overrides=deep_merge(C2, {"drone": {"ideal_actuator": True}} if ideal else {}))
    sim = Simulator(cfg, cfg["rates"]["rate_loop_hz"])
    p = params(cfg)
    c = GeometricController(p, Gains.from_bandwidths(p), sim.control_dt)
    tr = T.hover(get_state(sim.model, sim.data).x_L)
    sim.data.qvel[6:9] = [0.0, 0.0, v_up]
    mujoco.mj_forward(sim.model, sim.data)
    log, lengths = Logger(), []
    while sim.data.time < duration - 1e-9:
        s = get_state(sim.model, sim.data)
        r = tr(s.t)
        cmd = c.load_position(s, r, flat_state(r, p))
        want = sim.mixer.unconstrained(cmd)
        got = sim.mixer(cmd)
        log.log(s, tension=cable_tension(sim.model, sim.data, p.g), ref=r, wrench=cmd, rotor_cmd=got,
                rotor_unconstrained=want)
        lengths.append(s.cable_length)
        sim.step_rotor(got)
    L = log.arrays()
    m = compute_metrics(L, sim.control_dt, t_end=0.0, pendulum_period=2 * np.pi * np.sqrt(p.l / p.g))
    m["slack_time_s"] = float(np.sum(np.array(lengths) < p.l - 1e-4) * sim.control_dt)
    return m, L, np.array(lengths)


def main():
    run = RunRecorder("phase2", "e26a_paper1_on_c2", load_config(overrides=C2),
                      description="Step 2.6a E1.4a: Paper 1 controller on slack-capable cable C2 (dt 0.5 ms). Recovery "
                                  "(2.4), benchmark T0-T3 (2.5), T5(a) payload thrown up 3 m/s.")
    m, data = {}, {}
    # recovery
    E1 = R.experiment_1(extra=C2)
    E2 = R.experiment_2(extra=C2)
    for k in KEYS:
        m[f"rec.E1_178_ideal.{k}"] = E1[k]
    m["rec.E1_178_ideal.passed_paper1_criteria"] = E1["passed"]
    for a, r in E2["runs"].items():
        for k in KEYS:
            m[f"rec.E2_x2.theta{a:g}.{k}"] = r[k]
    m["rec.E2_x2.recovered_angles"] = [a for a, r in E2["runs"].items() if r["recovered"]]
    for k, v in E1["series"].items():
        data[f"rec_E1_{k}"] = v
    # benchmark
    bench = B.run_all(extra=C2)
    for name, v in bench["cases"].items():
        for act in ("x2", "ideal"):
            for k, val in v[act].items():
                m[f"bench.{name}.{act}.{k}"] = val
    md, csv = B.table(bench)
    (run.dir / "results_table_C2.md").write_text(md + "\n")
    (run.dir / "results_table_C2.csv").write_text(csv + "\n")
    # T5(a)
    t5 = {}
    for ideal in (False, True):
        mm, L, lens = t5_throw(ideal)
        act = "ideal" if ideal else "x2"
        t5[act] = (mm, L, lens)
        for k, val in mm.items():
            m[f"T5a.{act}.{k}"] = val
        data[f"T5a_{act}_t"], data[f"T5a_{act}_len"], data[f"T5a_{act}_T"] = L["t"], lens, L["T"]
        data[f"T5a_{act}_err"] = np.linalg.norm(L["x_L"] - L["ref"][:, 0, :], axis=1)
    finished = bench["passed"] and E2["passed"] and all(np.all(np.isfinite(v[1]["x_L"])) for v in t5.values())
    m["all_passed"] = bool(finished)
    run.save_metrics(m)
    run.save_data(**data)

    fig, ax = plt.subplots(1, 4, figsize=(21, 4.2))
    S = E1["series"]
    ax[0].semilogy(S["t"], S["e_x"]); ax[0].set(title="C2, 178° ideal actuator: |e_x| [m]", xlabel="t [s]")
    ax[1].plot(S["t"], S["tension"]); ax[1].set(title="C2, 178°: tension [N] (≥ 0, slack = 0)", xlabel="t [s]")
    angs = np.array(list(E2["runs"]))
    rec = np.array([E2["runs"][a]["recovered"] for a in angs])
    ax[2].scatter(angs[rec], [E2["runs"][a]["t_recover_s"] for a in angs[rec]], c="g", label="recovered")
    ax[2].scatter(angs[~rec], np.full((~rec).sum(), 10.5), c="r", marker="x", label="not in 10 s")
    ax[2].set(title="C2, X2 flip sweep", xlabel="θ [deg]", ylabel="t recover [s]"); ax[2].legend(fontsize=8)
    for act, st in (("x2", "-"), ("ideal", "--")):
        mm, L, lens = t5[act]
        ax[3].plot(L["t"], lens, st, label=f"length {act}")
    ax[3].axhline(0.5, color="r", lw=0.8)
    ax[3].set(title="T5(a): payload thrown up 3 m/s, cable length [m]", xlabel="t [s]"); ax[3].legend(fontsize=8)
    fig.tight_layout()
    run.save_figure(fig, "paper1_on_c2")
    print(run.finish(notes="Reported, not gated. Compare with e24_recovery and e25_benchmark_subset (C1)."))
    print("E1 178 ideal:", {k: E1[k] for k in KEYS}, "paper1 criteria", E1["passed"])
    for a, r in E2["runs"].items():
        print(f"E2 {a:g}: rec {r['recovered']} t {r['t_recover_s']:.2f} drop {r['height_loss_m']:.1f} sat {r['sat_frac']:.2f}")
    print(md)
    for act, (mm, _, _) in t5.items():
        print("T5a", act, {k: mm[k] for k in ("payload_max_err", "settling_time_s", "slack_time_s", "slack_fraction",
                                              "swing_max_deg", "success", "payload_above_drone")})
    print("ALL", finished)


if __name__ == "__main__":
    main()

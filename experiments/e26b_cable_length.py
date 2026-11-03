"""Step 2.6b (E1.4b): Paper 1 controller with cable lengths 0.25 m and 1.0 m (C1, gains unchanged 1.5/7.5/30 rad/s;
k_q = m_Q l w_q^2 scales with l). Benchmark T0-T3 (as step 2.5, X2 + ideal) and the X2 feasibility boundary T0* of
the main ellipse (as step 2.3). 0.5 m reference: e25_benchmark_subset, T0* = 2.121 s. Agreed: report only, no retuning.
Gate: every run finishes numerically and the hover tension equals m_L g at each length."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np

from sim import benchmark as B
from sim import study_ellipse as S
from sim.actuators import Mixer
from sim.checks_flatness import params
from sim.config import load_config
from sim.model import build_model, drone_info
from sim.run_io import RunRecorder

LENGTHS = (0.25, 0.5, 1.0)


def main():
    run = RunRecorder("phase2", "e26b_cable_length", load_config(),
                      description="Step 2.6b: Paper 1 controller, C1 cable length 0.25 / 0.5 / 1.0 m, benchmark T0-T3 "
                                  "(X2 + ideal) and ellipse feasibility boundary T0*.")
    m, ok = {}, True
    for L in LENGTHS:
        extra = {"cable": {"length": L}}
        cfg = load_config(overrides=extra)
        p = params(cfg)
        w = float(np.sqrt(p.g * (p.m_Q + p.m_L) / (p.m_Q * L)))
        m[f"L{L}.swing_omega_rad_s"] = w
        m[f"L{L}.T0_crit_main_s"] = S.critical_period((0.5, 1.0), p, Mixer(drone_info(build_model(cfg)[0])),
                                                      float(cfg["sim"]["start_altitude"] - L))
        bench = B.run_all(extra=extra)
        ok &= bench["passed"]
        hover_T = bench["cases"]["T1_4s"]["log_x2"]["T"][1:200]
        hov_err = float(np.abs(hover_T - p.m_L * p.g).max())
        m[f"L{L}.hover_tension_err_N"] = hov_err
        ok &= hov_err < 1e-3
        for name, v in bench["cases"].items():
            for act in ("x2", "ideal"):
                for k, val in v[act].items():
                    m[f"L{L}.{name}.{act}.{k}"] = val
        md, csv = B.table(bench)
        (run.dir / f"results_table_L{L}.md").write_text(md + "\n")
        (run.dir / f"results_table_L{L}.csv").write_text(csv + "\n")
        print(f"\n### L = {L} m  (swing ω = {w:.2f} rad/s, T0* main ellipse = {m[f'L{L}.T0_crit_main_s']:.3f} s, "
              f"hover tension err {hov_err:.1e} N)\n{md}", flush=True)
    m["all_passed"] = bool(ok)
    run.save_metrics(m)
    print(run.finish(notes="Reported, not gated (agreed: no retuning; D12 in Phase 3)."))
    print("ALL", ok)


if __name__ == "__main__":
    main()

"""Step 2.3 (E1.1): Paper 1 flatness study on the ellipse. Part A reproduces Fig. 3, Part B finds the X2's feasibility
boundary T0*, Part C runs the closed loop on the X2 vs the ideal actuator (3 periods per T0). See sim/study_ellipse.py."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from sim import study_ellipse as S
from sim.config import load_config
from sim.run_io import RunRecorder


def main():
    cfg = load_config()
    run = RunRecorder("phase2", "e23_ellipse_study", cfg,
                      description="Step 2.3 E1.1: Paper 1 ellipse x_L=(Ax sin, Ay cos, Az), psi=0, sizes 0.5x1.0 and "
                                  "1.0x2.0 m. A: Fig. 3 reproduction; B: X2 feasibility T0*; C: closed loop X2 vs ideal.")
    A = S.part_a(cfg)
    B = S.part_b(cfg)
    C = S.part_c(B["T0_crit"])

    m, data = {}, {}
    for name, v in A["sizes"].items():
        m[f"A.{name}.load_path_err"] = v["load_path_err"]
        for T0, r in v["rows"].items():
            for k in ("rotor_max", "rotor_min", "M_max", "f_min", "f_max", "tilt_max_deg", "quad_radius_max"):
                m[f"A.{name}.T{T0}.{k}"] = r[k]
            data[f"A_{name}_T{T0}_xQ"] = r["x_Q"]
            data[f"A_{name}_T{T0}_xL"] = r["x_L"]
    m["A.passed"] = A["passed"]
    for name, v in B["T0_crit"].items():
        m[f"B.T0_crit.{name}"] = v
        m[f"B.load_v_peak_at_crit.{name}"] = B["load_at_crit"][name]["v_peak"]
        m[f"B.load_a_peak_at_crit.{name}"] = B["load_at_crit"][name]["a_peak"]
    for s, v in B["sweep"].items():
        m[f"B.sweep.scale{s}"] = v
    m["B.passed"] = B["passed"]
    for name, res in C["sizes"].items():
        for T0, r in res.items():
            for act in ("ideal", "x2"):
                for k in ("rmse", "max_err", "sat_frac", "diverged"):
                    m[f"C.{name}.T{T0}.{act}.{k}"] = r[act][k]
                data[f"C_{name}_T{T0}_{act}_t"] = r[act]["t"]
                data[f"C_{name}_T{T0}_{act}_err"] = r[act]["err"]
    m["C.passed"] = C["passed"]
    m["all_passed"] = bool(A["passed"] and B["passed"] and C["passed"])
    run.save_metrics(m)
    run.save_data(**data)

    # Fig. 1: Paper 1 Fig. 3 reproduction (main size): drone paths (colour) for one load ellipse (black)
    main = A["sizes"]["main_0.5x1.0"]["rows"]
    fig = plt.figure(figsize=(16, 5))
    ax3 = fig.add_subplot(1, 3, 1, projection="3d")
    ax2 = fig.add_subplot(1, 3, 2)
    axz = fig.add_subplot(1, 3, 3)
    cmap = plt.get_cmap("viridis")
    for i, (T0, r) in enumerate(main.items()):
        col = cmap(i / (len(main) - 1))
        xq = np.vstack([r["x_Q"], r["x_Q"][:1]])
        ax3.plot(*xq.T, color=col, label=f"T0={T0:g} s")
        ax2.plot(xq[:, 0], xq[:, 1], color=col)
        axz.plot(xq[:, 1], xq[:, 2], color=col)
    xl = np.vstack([main[10.0]["x_L"], main[10.0]["x_L"][:1]])
    for a, (i, j) in ((ax2, (0, 1)), (axz, (1, 2))):
        a.plot(xl[:, i], xl[:, j], "k", lw=2, label="load")
        a.set_aspect("equal")
    ax3.plot(*xl.T, "k", lw=2)
    ax3.set(title="drone paths for one load ellipse (0.5x1.0 m)", xlabel="x", ylabel="y", zlabel="z")
    ax3.legend(fontsize=7)
    ax2.set(title="top view", xlabel="x [m]", ylabel="y [m]")
    axz.set(title="side view", xlabel="y [m]", ylabel="z [m]")
    fig.tight_layout()
    run.save_figure(fig, "A_fig3_reproduction")

    # Fig. 2: demand vs T0 and feasibility boundary
    fig, ax = plt.subplots(1, 3, figsize=(17, 4.5))
    for name, v in A["sizes"].items():
        Ts = np.array(list(v["rows"]))
        ax[0].semilogy(Ts, [v["rows"][T]["rotor_max"] for T in Ts], "o-", label=f"{name} max")
        ax[0].semilogy(Ts, [max(-v["rows"][T]["rotor_min"], 1e-3) for T in Ts], "x--", label=f"{name} -min (if <0)")
        ax[1].plot(Ts, [v["rows"][T]["tilt_max_deg"] for T in Ts], "o-", label=name)
        ax[0].axvline(B["T0_crit"][name], color="gray", ls=":", lw=1)
    ax[0].axhline(13.0, color="r", lw=1, label="X2 limit 13 N")
    ax[0].set(title="flatness rotor thrust demand", xlabel="T0 [s]", ylabel="[N]")
    ax[0].legend(fontsize=7)
    ax[1].axhline(90, color="r", lw=1)
    ax[1].set(title="required peak tilt", xlabel="T0 [s]", ylabel="[deg]")
    ax[1].legend(fontsize=8)
    sc = np.array(list(B["sweep"]))
    ax[2].plot(sc * 1.0, [B["sweep"][s] for s in sc], "o-")
    ax[2].set(title="X2 feasibility boundary T0* (Ax:Ay = 1:2)", xlabel="A_y [m]", ylabel="T0* [s]")
    fig.tight_layout()
    run.save_figure(fig, "B_feasibility")

    # Fig. 3: closed loop
    fig, ax = plt.subplots(1, 2, figsize=(13, 4.5))
    for i, (name, res) in enumerate(C["sizes"].items()):
        Ts = np.array(list(res))
        for act, mk in (("ideal", "o-"), ("x2", "s--")):
            ax[i].semilogy(Ts, [res[T][act]["max_err"] for T in Ts], mk, label=act)
        ax[i].axvline(B["T0_crit"][name], color="gray", ls=":", label="T0* (X2)")
        ax[i].set(title=f"closed loop, {name}: max load error over 3 periods", xlabel="T0 [s]", ylabel="[m]")
        ax[i].legend(fontsize=8)
    fig.tight_layout()
    run.save_figure(fig, "C_closed_loop")

    out = run.finish(notes="Gate passes only if all_passed is true. Ideal-actuator tracking gated for T0 >= 2 s; T0 = 1.5 s "
                           "reported (controller bandwidth limit, F4).")
    print("A", A["passed"], {n: (v["tilt_monotone"], v["torque_monotone"]) for n, v in A["sizes"].items()})
    print("B", B["passed"], B["T0_crit"], B["sweep"])
    for name, res in C["sizes"].items():
        for T0, r in res.items():
            print(f"C {name} T0={T0}: ideal max {r['ideal']['max_err']:.2e}  x2 max {r['x2']['max_err']:.2e} "
                  f"sat {r['x2']['sat_frac']:.2f} div {r['x2']['diverged']}")
    print("C", C["passed"], "ALL", m["all_passed"])
    print(out)


if __name__ == "__main__":
    main()

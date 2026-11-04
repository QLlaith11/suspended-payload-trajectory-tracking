"""Step 2.6c-2 (E1.4c): Paper 1 controller under realistic actuation and sensing, one factor at a time (D8, D10).
Cases: the 11 flat-feasible T0-T3 benchmark cases (C1 0.5 m, X2). Settings:
  base   CTBM 500 Hz, L0               (= step 2.5)
  lag    + motor lag 35 ms             noise  + sensor noise (20 seeds)
  delay  + action delay 10 ms          drag   + air drag
  ctbr   CTBR, outer 100 Hz, rate loop 500 Hz
  all    CTBR 100 Hz + lag + delay + noise + drag (full L1, 20 seeds)
Gain sets: nominal 1.5/7.5/30 rad/s (F3) and lag-compatible 0.4/2/10 rad/s (step 2.6c linear analysis; agreed test of
the bandwidth conflict, not tuning). Reported, not gated; all_passed = every run finished.
Usage: python3 experiments/e26c2_realism_ofat.py [--quick] [--seeds=N] [--workers=N (default 2)]   (--quick: 2 seeds, smoke test; default 20)"""
import json
import sys
import time
from multiprocessing import Pool
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np

from sim import benchmark as B
from sim.closed_loop import run
from sim.config import load_config
from sim.model import build_model
from sim.run_io import RunRecorder
from sim.state import get_state

SETTINGS = {
    "base": ({}, "CTBM", None, False),
    "lag": ({"drone": {"motor_tau": 0.035}}, "CTBM", None, False),
    "delay": ({"realism": {"action_delay": 0.01}}, "CTBM", None, False),
    "noise": ({"realism": {"sensor_noise": True}}, "CTBM", None, True),
    "drag": ({"realism": {"fluid_drag": True}}, "CTBM", None, False),
    "ctbr": ({}, "CTBR", 100, False),
    "all": ({"drone": {"motor_tau": 0.035},
             "realism": {"action_delay": 0.01, "sensor_noise": True, "fluid_drag": True}}, "CTBR", 100, True),
}
GAINS = {"nominal": (1.5, 7.5, 30.0), "lagcompat": (0.4, 2.0, 10.0)}
EXCLUDE = {"T0_gust_severe", "T1_1s"}          # not flat-feasible / severe level (step 2.5)
KEYS = ("payload_rmse", "payload_max_err", "swing_max_deg", "dynamic_swing_max_deg", "settling_time_s",
        "saturation_fraction", "tilt_max_deg", "success", "crashed")


def _cases():
    x0 = get_state(*build_model(load_config())).x_L
    return {k: v for k, v in B.cases(x0).items() if k not in EXCLUDE}


def _job(args):
    case, setting, gset, seed = args
    tr, dur, sw, gust = _cases()[case]
    extra, iface, ohz, _ = SETTINGS[setting]
    gust_n = {"main": B.GUST_ACC * 0.2, "severe": B.GUST_SEVERE_N}.get(gust, 0.0)
    try:
        m, _ = run(tr, dur, extra=extra, interface=iface, outer_hz=ohz, seed=seed, swing0=sw, gust_n=gust_n,
                   bandwidths=GAINS[gset], log_tension=False)
        out = {k: m.get(k) for k in KEYS}
        out["finished"] = True
    except Exception as e:                         # a numerical failure is a result, recorded as such
        out = {"finished": False, "error": repr(e)}
    return (case, setting, gset, seed), out


def main():
    quick = "--quick" in sys.argv
    n_seeds = 2 if quick else next((int(a.split("=")[1]) for a in sys.argv if a.startswith("--seeds=")), 20)
    run_rec = RunRecorder("phase2", "e26c2_realism_ofat", load_config(),
                          description="Step 2.6c-2: Paper 1 controller, one-factor-at-a-time L1 realism (lag, delay, "
                                      "noise, drag, CTBR, all), gain sets nominal 1.5/7.5/30 and lag-compatible "
                                      f"0.4/2/10, 11 feasible T0-T3 cases, {n_seeds} seeds for noisy settings.")
    jobs = [(c, s, g, seed) for c in _cases() for s, v in SETTINGS.items() for g in GAINS
            for seed in (range(n_seeds) if v[3] else [0])]
    print(len(jobs), "runs", flush=True)
    # checkpoint: every finished run is appended to a file shared by all attempts; a restart skips finished runs
    ckpt = run_rec.dir.parent / f"checkpoint_seeds{n_seeds}.jsonl"
    res = {}
    if ckpt.exists():
        for line in ckpt.read_text().splitlines():
            if line.strip():
                rec = json.loads(line)
                res[tuple(rec["key"][:3]) + (int(rec["key"][3]),)] = rec["out"]
    todo = [j for j in jobs if j not in res]
    print(f"resuming: {len(res)} runs already done, {len(todo)} to go (checkpoint {ckpt.name})", flush=True)
    t0 = time.time()
    workers = next((int(a.split("=")[1]) for a in sys.argv if a.startswith("--workers=")), 2)
    print("workers:", workers, flush=True)
    with Pool(workers) as pool, open(ckpt, "a") as f:
        for i, (k, v) in enumerate(pool.imap_unordered(_job, todo, chunksize=1), 1):
            res[k] = v
            f.write(json.dumps({"key": list(k), "out": v}, default=float) + "\n")
            f.flush()
            if i % 10 == 0 or i == len(todo):
                el = time.time() - t0
                print(f"progress {len(res)}/{len(jobs)}  elapsed {el / 60:.1f} min  "
                      f"remaining ~{el / i * (len(todo) - i) / 60:.0f} min", flush=True)
    res = {j: res[j] for j in jobs}

    raw = {"|".join(map(str, k)): v for k, v in res.items()}
    (run_rec.dir / "runs.json").write_text(json.dumps(raw, indent=0, default=float))
    summary, lines = {}, []
    hdr = "| gains | setting | success | max err median [m] (20–80 %) | worst case | crashed runs |"
    lines += [hdr, "|---|---|---|---|---|---|"]
    for g in GAINS:
        for s in SETTINGS:
            rs = {k: v for k, v in res.items() if k[1] == s and k[2] == g}
            fin = [v for v in rs.values() if v["finished"]]
            succ = np.mean([bool(v["success"]) for v in fin]) if fin else 0.0
            errs = np.array([v["payload_max_err"] for v in fin])
            crashed = sum(bool(v["crashed"]) for v in fin) + (len(rs) - len(fin))
            worst = max(fin, key=lambda v: v["payload_max_err"]) if fin else None
            worst_case = [k[0] for k, v in rs.items() if v is worst][0] if worst else "-"
            summary[f"{g}.{s}"] = {"runs": len(rs), "success_rate": float(succ), "crashed": int(crashed),
                                   "max_err_median": float(np.median(errs)), "max_err_p20": float(np.percentile(errs, 20)),
                                   "max_err_p80": float(np.percentile(errs, 80)), "worst_case": worst_case}
            lines.append(f"| {g} | {s} | {succ:.0%} | {np.median(errs):.2e} ({np.percentile(errs, 20):.1e}–"
                         f"{np.percentile(errs, 80):.1e}) | {worst_case} | {crashed}/{len(rs)} |")
    # per-case success table (success rate over seeds)
    cases = list(_cases())
    lines += ["", "| gains | setting | " + " | ".join(cases) + " |", "|---|---|" + "---|" * len(cases)]
    for g in GAINS:
        for s in SETTINGS:
            row = []
            for c in cases:
                vs = [v for k, v in res.items() if k[0] == c and k[1] == s and k[2] == g]
                sr = np.mean([v["finished"] and bool(v["success"]) for v in vs])
                me = np.median([v["payload_max_err"] if v["finished"] else np.inf for v in vs])
                row.append(f"{sr:.0%} / {me:.1e}")
            lines.append(f"| {g} | {s} | " + " | ".join(row) + " |")
    table = "\n".join(lines)
    (run_rec.dir / "results_table.md").write_text(
        "Cells of the second table: success rate / median max payload error [m].\n\n" + table + "\n")
    summary["all_passed"] = all(v["finished"] for v in res.values())
    run_rec.save_metrics(summary)
    print(run_rec.finish(notes="Reported, not gated. runs.json holds every run; results_table.md the summaries."))
    print(table)
    print("ALL finished:", summary["all_passed"])


if __name__ == "__main__":
    main()

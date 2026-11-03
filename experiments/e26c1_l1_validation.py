"""Step 2.6c-1 gate: L1 realism stack (sensor noise, action delay, air drag) + CTBR rate loop + closed-loop runner.
Gates R1-R8, see sim/checks_l1.py."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sim import checks_l1 as C
from sim.config import load_config
from sim.run_io import RunRecorder


def main():
    run = RunRecorder("phase2", "e26c1_l1_validation", load_config(),
                      description="Step 2.6c-1: L1 realism + CTBR validation, gates R1-R8.")
    res = {n: f() for n, f in C.ALL_CHECKS.items()}
    m = {f"{n}.{k}": v for n, r in res.items() for k, v in r.items()}
    m["all_passed"] = all(r["passed"] for r in res.values())
    run.save_metrics(m)
    print(run.finish(notes="Gate passes only if all_passed is true."))
    for n, r in res.items():
        print(n, "PASS" if r["passed"] else "FAIL")


if __name__ == "__main__":
    main()

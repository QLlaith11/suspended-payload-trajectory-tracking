import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sim import benchmark as B


def test_benchmark_runs_and_flags_infeasible():
    r = B.run_all(names={"T1_2s", "T1_1s"})
    assert r["passed"]
    assert r["cases"]["T1_2s"]["feasibility"]["feasible"] and r["cases"]["T1_2s"]["x2"]["success"]
    assert not r["cases"]["T1_1s"]["feasibility"]["feasible"]

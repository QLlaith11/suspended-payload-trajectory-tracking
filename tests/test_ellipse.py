import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sim import study_ellipse as S


def test_part_a_fig3_trends():
    assert S.part_a()["passed"]


def test_part_b_feasibility_boundary():
    b = S.part_b(scales=(0.5, 1.0, 2.0))
    assert b["pre_check_ok"] and b["sweep_monotone"]


def test_part_c_x2_matches_ideal_with_margin():
    c = S.part_c({"main_0.5x1.0": 2.121, "second_1.0x2.0": 2.610}, periods=(3.0,))
    assert c["passed"]

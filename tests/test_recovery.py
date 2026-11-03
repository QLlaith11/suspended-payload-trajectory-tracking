import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from sim import study_recovery as R


def test_paper1_recovery_178_ideal_actuator():
    assert R.experiment_1()["passed"]


def test_x2_small_flip_recovers():
    r = R.run_recovery(10.0, ideal=False)
    assert r["recovered"] and r["height_loss_m"] < 0.1

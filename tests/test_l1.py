import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import pytest

from sim import checks_l1


@pytest.mark.parametrize("name", list(checks_l1.ALL_CHECKS))
def test_l1_gates(name):
    r = checks_l1.ALL_CHECKS[name]()
    assert r["passed"], (name, r)

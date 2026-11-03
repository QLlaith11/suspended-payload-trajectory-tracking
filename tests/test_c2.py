import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import pytest

from sim import checks_c2


@pytest.mark.parametrize("name", list(checks_c2.ALL_CHECKS))
def test_c2_gates(name):
    r = checks_c2.ALL_CHECKS[name]()
    assert r["passed"], (name, {k: v for k, v in r.items() if k != "series"})

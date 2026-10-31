import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml

from sim.config import deep_merge, load_config
from sim.run_io import RunRecorder


def test_deep_merge_keeps_unrelated_keys():
    base = {"a": {"x": 1, "y": 2}, "b": 3}
    out = deep_merge(base, {"a": {"y": 5}})
    assert out == {"a": {"x": 1, "y": 5}, "b": 3}
    assert base["a"]["y"] == 2  # base not mutated


def test_base_config_values_match_design():
    cfg = load_config()
    assert cfg["sim"]["dt"] == 0.001
    assert cfg["rates"]["outer_loop_hz"] == 100
    assert cfg["cable"]["EA"] == 125.0
    assert cfg["payload"]["mass"] == 0.2


def test_run_recorder_creates_complete_folder(tmp_path):
    cfg = load_config()
    run = RunRecorder("phase_test", "exp", cfg, description="test", results_root=tmp_path)
    run.save_data(x=np.arange(3))
    run.save_metrics({"m": 1.0})
    fig, ax = plt.subplots()
    ax.plot([0, 1])
    run.save_figure(fig, "fig")
    out = run.finish()
    for name in ["config.yaml", "meta.json", "metrics.json", "data.npz", "fig.png", "README.md"]:
        assert (out / name).exists(), name
    assert yaml.safe_load((out / "config.yaml").read_text()) == cfg
    meta = json.loads((out / "meta.json").read_text())
    assert "commit" in meta["git"] and "mujoco" in meta["versions"]

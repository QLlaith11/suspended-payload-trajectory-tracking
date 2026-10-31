"""Run recording: every experiment run gets its own folder with everything needed to reproduce it.

results/<phase>/<experiment>/<YYYYmmdd-HHMMSS>/
    config.yaml     full configuration used
    meta.json       git commit, dirty flag, versions, command, timestamps
    metrics.json    scalar results
    data.npz        raw time series (not versioned in git)
    *.png           figures (not versioned in git)
    README.md       human-readable summary
"""
from __future__ import annotations

import json
import platform
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import yaml

from .config import ROOT


def _git(*args: str) -> str:
    try:
        return subprocess.run(["git", *args], cwd=ROOT, capture_output=True, text=True, check=True).stdout.strip()
    except (subprocess.CalledProcessError, FileNotFoundError):
        return ""


def _to_builtin(value):
    """Convert numpy scalars/arrays to plain Python types so they can be written as JSON."""
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, np.ndarray):
        return value.tolist()
    return value


def git_info() -> dict:
    commit = _git("rev-parse", "HEAD")
    dirty = bool(_git("status", "--porcelain", "--untracked-files=no"))
    return {"commit": commit or "no-commit-yet", "dirty": dirty}


def versions() -> dict:
    import matplotlib
    import mujoco
    import scipy

    return {
        "python": platform.python_version(),
        "mujoco": mujoco.__version__,
        "numpy": np.__version__,
        "scipy": scipy.__version__,
        "matplotlib": matplotlib.__version__,
        "platform": platform.platform(),
    }


class RunRecorder:
    def __init__(self, phase: str, experiment: str, config: dict, description: str = "",
                 results_root: Path | None = None):
        self.phase, self.experiment, self.config = phase, experiment, config
        self.description = description
        self.start = time.time()
        stamp = time.strftime("%Y%m%d-%H%M%S")
        root = Path(results_root) if results_root else ROOT / "results"
        self.dir = root / phase / experiment / stamp
        suffix = 1
        while self.dir.exists():  # two runs in the same second
            self.dir = root / phase / experiment / f"{stamp}-{suffix}"
            suffix += 1
        self.dir.mkdir(parents=True)
        self.metrics: dict = {}
        self.figures: list[str] = []
        self.meta = {
            "phase": phase,
            "experiment": experiment,
            "git": git_info(),
            "versions": versions(),
            "command": " ".join(sys.argv),
            "started": time.strftime("%Y-%m-%d %H:%M:%S"),
        }
        with open(self.dir / "config.yaml", "w") as f:
            yaml.safe_dump(config, f, sort_keys=False)

    def save_data(self, **arrays) -> None:
        np.savez_compressed(self.dir / "data.npz", **arrays)

    def save_metrics(self, metrics: dict) -> None:
        self.metrics.update({k: _to_builtin(v) for k, v in metrics.items()})
        with open(self.dir / "metrics.json", "w") as f:
            json.dump(self.metrics, f, indent=2)

    def save_figure(self, fig, name: str) -> Path:
        path = self.dir / f"{name}.png"
        fig.savefig(path, dpi=120, bbox_inches="tight")
        self.figures.append(path.name)
        return path

    def finish(self, notes: str = "") -> Path:
        self.meta["finished"] = time.strftime("%Y-%m-%d %H:%M:%S")
        self.meta["wall_time_s"] = round(time.time() - self.start, 3)
        with open(self.dir / "meta.json", "w") as f:
            json.dump(self.meta, f, indent=2)
        g = self.meta["git"]
        lines = [
            f"# {self.phase} / {self.experiment}",
            "",
            self.description,
            "",
            f"- Started: {self.meta['started']}  (wall time {self.meta['wall_time_s']} s)",
            f"- Code: commit `{g['commit']}`" + ("  **(uncommitted changes present)**" if g["dirty"] else ""),
            f"- Command: `{self.meta['command']}`",
            f"- MuJoCo {self.meta['versions']['mujoco']}, Python {self.meta['versions']['python']}",
            "",
            "## Metrics",
            "",
            "| metric | value |",
            "|---|---|",
            *[f"| {k} | {v} |" for k, v in self.metrics.items()],
            "",
            "## Figures",
            "",
            *[f"- {name}" for name in self.figures],
        ]
        if notes:
            lines += ["", "## Notes", "", notes]
        (self.dir / "README.md").write_text("\n".join(lines) + "\n")
        return self.dir

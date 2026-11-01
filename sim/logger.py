"""Per-control-tick logging of state, reference and commands (step 1.4)."""
from __future__ import annotations

from pathlib import Path

import numpy as np

from .state import State


class Logger:
    def __init__(self):
        self._rec: dict[str, list] = {}

    def _add(self, key: str, value) -> None:
        self._rec.setdefault(key, []).append(np.array(value, dtype=float) if not isinstance(value, bool)
                                             else value)

    def log(self, state: State, tension: float | None = None, ref: np.ndarray | None = None,
            wrench: np.ndarray | None = None, rotor_cmd: np.ndarray | None = None,
            rotor_unconstrained: np.ndarray | None = None, ctrl_time: float | None = None, **extra) -> None:
        """Record one control tick. `ref` is the (7, 3) payload reference; `rotor_unconstrained` is the mixer
        output before saturation (to flag saturation)."""
        self._add("t", state.t)
        for k in ("x_Q", "v_Q", "Omega", "x_L", "v_L", "q", "q_dot", "omega"):
            v = getattr(state, k)
            if v is not None:
                self._add(k, v)
        self._add("R", state.R.reshape(9))
        if tension is not None:
            self._add("T", tension)
        if ref is not None:
            self._add("ref", ref)
        if wrench is not None:
            self._add("wrench", wrench)
        if rotor_cmd is not None:
            self._add("rotor_cmd", rotor_cmd)
            if rotor_unconstrained is not None:
                self._add("saturated", float(not np.allclose(rotor_cmd, rotor_unconstrained, atol=1e-9)))
        if ctrl_time is not None:
            self._add("ctrl_time", ctrl_time)
        for k, v in extra.items():
            self._add(k, v)

    def arrays(self) -> dict[str, np.ndarray]:
        return {k: np.array(v) for k, v in self._rec.items()}

    def save(self, path: Path) -> None:
        np.savez_compressed(path, **self.arrays())

    @staticmethod
    def load(path: Path) -> dict[str, np.ndarray]:
        with np.load(path) as f:
            return {k: f[k] for k in f.files}

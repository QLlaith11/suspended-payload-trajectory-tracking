"""Configuration loading: base config + experiment overrides (deep merge)."""
from __future__ import annotations

import copy
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
BASE_CONFIG = ROOT / "configs" / "base.yaml"


def deep_merge(base: dict, override: dict) -> dict:
    """Return a new dict: `base` recursively updated with `override`."""
    out = copy.deepcopy(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(out.get(key), dict):
            out[key] = deep_merge(out[key], value)
        else:
            out[key] = copy.deepcopy(value)
    return out


def load_config(experiment_config: str | Path | None = None, overrides: dict | None = None) -> dict:
    """Load base.yaml, then an optional experiment YAML, then optional in-code overrides."""
    with open(BASE_CONFIG) as f:
        cfg = yaml.safe_load(f)
    if experiment_config is not None:
        with open(experiment_config) as f:
            cfg = deep_merge(cfg, yaml.safe_load(f) or {})
    if overrides:
        cfg = deep_merge(cfg, overrides)
    return cfg

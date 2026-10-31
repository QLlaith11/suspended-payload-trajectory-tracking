#!/usr/bin/env bash
# Run the unit tests. Plugin autoload is disabled because some system-wide pytest plugins
# (anyio, typeguard) are incompatible with the system pytest 6.2.5.
cd "$(dirname "$0")/.." && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -q tests "$@"

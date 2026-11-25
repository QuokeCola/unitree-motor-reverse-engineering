#!/bin/sh
# Launch the G1 arm motor monitor. Extra arguments are passed through (e.g. --replay FILE.bin).
cd "$(dirname "$0")" && exec .venv/bin/python scripts/monitor.py "$@"

#!/bin/sh
# Launch the live bus monitor. Extra arguments are passed through (e.g. --replay FILE.bin).
# Uses the virtual environment at the top of the repository if there is one.
here="$(cd "$(dirname "$0")" && pwd)"
python="$here/../../.venv/bin/python"
[ -x "$python" ] || python=python3
exec "$python" "$here/monitor.py" "$@"

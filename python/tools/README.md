# Research tools

Scripts used while reverse engineering the bus. They are separate from the SDK package: they have
their own small frame code (`g1bus.py`) and extra dependencies (`requirements.txt`: numpy,
pyqtgraph, PySide6). Only `verify_bench.py` uses the `g1motor` package.

| Script | Purpose |
|---|---|
| `sniff.py` | Record raw bus bytes, listen only |
| `monitor.py` (`monitor.sh`) | Live plots of one motor from a bus with a controller, or replay of a capture |
| `scan.py`, `zero_torque.py` | First bench tools: find motors, free the joints and record end stops |
| `sweep.py` | Motion tests: ramps and sines over the joint range, with logging and abort limits |
| `smoothness.py`, `build_ff_table.py` | Analyse a `sweep.py` log; build the friction feed-forward table |
| `mode_probe.py`, `mode_characterize.py`, `mode_coastdown.py`, `fault_clear_test.py` | Experiments that established the mode table and the fault reset |
| `verify_captures.py`, `verify_bench.py` | Re-test the statements of the protocol guide against captures and on the bench |

`calibration/` holds the end stops (`limits.json`, re-measure after every power-on) and the
feed-forward table. `captures/` holds raw recordings and logs; it is not tracked by git.

## Setup and operating modes

From `python/tools/`, install the research dependencies into your Python virtual
environment:

```sh
python -m pip install -r requirements.txt
```

Install the SDK separately for `verify_bench.py` with `python -m pip install -e ..`.
These research scripts may contain bench-specific port names, motor IDs, and
calibration paths. Read the script’s usage and defaults before running it.

- **Controller connected:** `sniff.py` records without transmitting; `monitor.py`
  observes traffic or replays a capture.
- **Controller disconnected:** active tools such as `scan.py`, `zero_torque.py`,
  `sweep.py`, and the mode experiments transmit commands. Use the
  [bench wiring instructions](../../README.md#wiring).
- **Offline:** capture verification and log analysis operate on recorded files;
  see each script for its required input paths.

The checked-in calibration files describe the test bench. Re-measure end stops
for your hardware after power-on; do not use those files as universal G1 limits.

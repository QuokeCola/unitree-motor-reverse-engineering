# Research tools

Scripts used while reverse engineering the bus. They are separate from the SDK package: they have
their own small frame code (`g1bus.py`) and extra dependencies (`requirements.txt`: numpy,
pyqtgraph, PySide6). Only `verify_bench.py` uses the SDK.

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

# Python interface

`g1_joint_motor_sdk` package and the `g1motor` command. Python 3.10+, needs `pyserial`.

    pip install -e .            # from this folder; add ".[dev]" for pytest
    g1motor scan

```python
from g1_joint_motor_sdk import MotorBus, load_joints

with MotorBus() as bus:                       # the only FTDI adapter, 6 Mbaud
    print(bus.scan())                         # {4: MotorState(...), ...}
    start = bus.enable([4, 5, 6])             # run mode, zero torque: joints are free
    for _ in range(1000):                     # hold joint 4 where it is
        state = bus.command(4, pos=start[4].pos, kp=300, kd=40)
# leaving the block returns the motors to idle, which brakes them
```

| Call | What it does |
|---|---|
| `MotorBus(port=None)` | Opens the adapter. Raises `BusBusyError` if the bus is not silent. |
| `bus.scan()` | Polls ids 0-14 with the idle frame; returns `{id: MotorState}`. |
| `bus.enable(ids)` | Run mode with zero torque. Raises `MotorFaultError` if a motor refuses. |
| `bus.command(id, pos, vel, tau, kp, kd)` | One hybrid command in raw counts; returns the fresh state. |
| `bus.read(id)` | State only. On an enabled motor this sends zero torque, which drops a hold. |
| `bus.disable(ids=None)` | Back to idle, which brakes the joint. Also done on leaving the `with` block. |
| `bus.restart(ids)` | Mode 7: clears a latched fault; about 0.3 s. Positions may come back shifted. |
| `bus.set_joint(id, joint)` | Attaches a `Joint`; targets are then limited to its end stops. |
| `Joint`, `load_joints(path)` | Gear ratio, end stops, software zero, output-shaft angles. |
| `MotorCmd`, `MotorState` | Frame encode and decode, usable on captured data. |
| `units` | Counts to rad, rad/s and N*m on the rotor side (torque scale assumed). |

Command line: `g1motor scan`, `read`, `limits` (record end stops by hand), `restart`, `sniff`.

The motor applies `tau + kp * (pos - measured) + kd * (vel - measured)` and keeps applying the last
command. Call `command()` for every enabled motor in a steady loop; a motor that is no longer
polled drops to idle after 0.5-1 s. End stops in counts must be re-checked after every power-on
(`g1motor limits`), because the position reading can shift by whole output turns.

## Tests and examples

    python -m pytest                    # protocol_test.py and joint_test.py, no hardware
    python tests/hardware_test.py       # motors on a bus without a controller

Examples are in `examples/`: `scan.py`, `zero_torque.py`, `position_hold.py`, `sine.py`.

## Research tools

`tools/` holds the scripts used to reverse engineer the bus (capture, live plots, motion tests,
verification of the protocol guide). They are separate from the package; see `tools/README.md`.

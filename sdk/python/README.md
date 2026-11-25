# g1-joint-motor-sdk (Python)

Unofficial Python driver for Unitree G1 joint motors, talking to them directly over their
RS-485 bus. Reverse engineered from bus captures on 2026-11-25; not affiliated with or
supported by Unitree. Version 0.2.0. Run on Linux; Windows and macOS paths are written but not run.
The C++ and MATLAB interfaces are in `../cpp` and `../matlab`; see `../README.md`.

Use it on a **bare arm or motor with no robot controller on the bus**. On a complete robot
the onboard controller is already the bus master, and a second master corrupts the traffic;
`MotorBus` refuses to open if it hears traffic. On a robot, use it only to listen.

## Install

    pip install -e .          # from this folder; needs pyserial
    pip install -e ".[dev]"   # plus pytest

Hardware: an FTDI FT232H RS-485 adapter (the Unitree one works) that supports 6 Mbaud. The
user needs permission on the serial device (group `uucp` or `dialout`).

## Quick start

```python
from g1_joint_motor_sdk import MotorBus

with MotorBus() as bus:                       # the only FTDI adapter, 6 Mbaud
    print(bus.scan())                         # {4: MotorState(...), 5: ..., 6: ...}
    start = bus.enable([4, 5, 6])             # run mode, zero torque: joints are free
    for _ in range(1000):                     # hold joint 4 where it is
        state = bus.command(4, pos=start[4].pos, kp=300, kd=40)
# leaving the block returns the motors to idle, which brakes them
```

Command line:

    g1motor scan
    g1motor read --ids 4,5,6 --seconds 10 [--zero-torque]
    g1motor restart --ids 4,5,6                       # clears a latched fault
    g1motor limits --ids 4,5,6 --out limits.json      # joints free: move them to both end stops
    g1motor sniff --seconds 30 --out capture.bin      # listen only, safe on a full robot

Examples: `examples/scan.py`, `zero_torque.py`, `position_hold.py`, `sine.py`.

## API

| Call | What it does |
|---|---|
| `MotorBus(port=None)` | Opens the adapter at 6 Mbaud. Raises `BusBusyError` if the bus is not silent. |
| `MotorState.error`, `.fault` | Status word from the motor; non-zero blocks run mode. |
| `bus.scan()` | Polls ids 0-14 with the idle frame; returns `{id: MotorState}`. |
| `bus.enable(ids)` | Idle -> run mode with zero torque, in the order the G1 controller uses. |
| `bus.command(id, pos, vel, tau, kp, kd)` | One hybrid command in raw counts; returns the fresh `MotorState`. |
| `bus.read(id)` | State only. On an enabled motor this sends zero torque, which drops a hold. |
| `bus.set_limits(id, low, high)` | Clamps position targets for that motor. |
| `bus.disable(ids=None)` | Back to idle, which brakes the joint passively. Also done on leaving the `with` block. |
| `bus.restart(ids)` | Restarts motors (mode 7): clears a latched fault; about 0.3 s. Positions may come back shifted. |
| `bus.listen(seconds)` | Raw bytes, no transmission. |
| `MotorCmd`, `MotorState` | Frame encode / decode, also usable on captured data. |
| `units` | Counts to rad, rad/s, N*m on the rotor side. Position and velocity scales are supported by data; torque is assumed. |
| `Joint(gear_ratio, low, high, zero)` | Output-shaft angles, end-stop clamp and a software zero. `sync(pos)` re-references after a restart. |
| `bus.set_joint(id, joint)` | `enable()` and `restart()` then re-reference the joint; `command()` clamps to its end stops. |
| `load_joints(path)` | Joints from a `g1motor limits` file, with the wrist gear ratios (16, 25, 25 for ids 4, 5, 6). |

The motor applies `tau + kp * (pos - measured) + kd * (vel - measured)` and keeps applying the
last command it received. Call `command()` for every enabled motor in a steady loop. A motor that
is no longer polled keeps going for 0.5-1 s before it drops to idle.

## Wire format

6,000,000 baud, 8N1, half duplex. The motor answers each command addressed to it.

Command, 20 bytes, little-endian:

| Offset | Size | Field |
|---|---|---|
| 0 | 2 | header `FE EE` |
| 2 | 1 | bits 0-3 id, bits 4-6 mode (0 idle, 1 run), bit 7 flag |
| 3 | 1 | toggles `80`/`90` in the controller's traffic |
| 4 | 2 | torque feed-forward, int16 |
| 6 | 2 | velocity target, int16 |
| 8 | 4 | position target, int32 |
| 12 | 2 | kp, int16 |
| 14 | 2 | kd, int16 |
| 16 | 4 | CRC-32 over bytes 0-15 |

Reply, 26 bytes:

| Offset | Size | Field |
|---|---|---|
| 0 | 2 | header `FC EE` |
| 2 | 1 | bits 0-3 id, bits 4-6 mode |
| 3 | 1 | coil (winding) temperature, deg C |
| 4 | 1 | shell temperature, deg C |
| 5 | 1 | bus voltage, 0.5 V per count |
| 6 | 2 | torque, int16 |
| 8 | 2 | velocity, int16 |
| 10 | 4 | position, int32 |
| 14 | 4 | status / fault word, 0 in normal running |
| 18 | 2 | unknown, about 2,600-3,050 |
| 20 | 2 | unknown |
| 22 | 4 | CRC-32 over bytes 2-21 (header excluded) |

CRC: polynomial `0x04C11DB7`, init `0xFFFFFFFF`, no reflection, no final XOR, over 32-bit
little-endian words (the `crc32_core` of Unitree's public A1/B1 SDK).

## What is verified and what is not

Verified on hardware:
- Frame layout and both checksums: about 3.8 million captured command/reply pairs from a G1 arm.
- `MotorBus.scan`, `enable`, `command` (zero torque and position hold), `disable`, `restart`, and `Joint`
  re-referencing through `set_joint`: run on the bench wrist.
- Scan, zero torque, position control with kp/kd, velocity and torque feed-forward: on a bench
  wrist (ids 4, 5, 6) at 24 V. Full-range sines up to a 0.8 s period.
- Those motors top out at about 1,275,000 position counts/s on 24 V.

Joint angles:

```python
from g1_joint_motor_sdk import MotorBus, load_joints

joints = load_joints("limits.json")              # end stops from `g1motor limits`
with MotorBus() as bus:
    for i, j in joints.items():
        bus.set_joint(i, j)
    state = bus.enable(sorted(joints))           # joints re-referenced to the motors' readings
    print(joints[4].angle(state[4].pos))         # rad at the output, from mid-travel
    bus.command(4, pos=joints[4].counts(0.3), kp=300, kd=40)
```

Not verified, or known gaps:
- **Physical units.** Position is 32,768 counts per rotor turn (supported by restart jumps of exactly
  16 and 25 turns); velocity follows from it. The torque scale is still assumed, and gear ratios are
  known only for the bench wrist (16:1 on id 4, 25:1 on id 6, id 5 assumed 25:1).
- **Zeroing.** The robot's calibration sets the zero with a mode-6 frame (captured, see the protocol guide).
  This SDK does not send it: it has not been reproduced and is presumably permanent. `Joint` keeps a zero in software.
- **Challenge and response.** Two of the robot's arm motors answer normally only after a 12-byte response that
  changes at every restart. The algorithm is unknown, so this SDK cannot bring those motors up.
- **Gain scale.** kp is raw; damping torque is 0.005 x kd x velocity reading (torque counts), up to the
  motor's torque limit. Tuned values for the bench wrist were kp 300, kd 40-80.
- **Other joints.** Only wrist motors were driven.
- **Byte 3 and the flag bit.** Reproduced as the controller sends them. Neither is needed to enter run mode.
- **Watchdog.** A motor that is no longer polled stays in run mode for 0.5-1 s, then drops to idle.
- **Faults.** `MotorState.error` is the status word. Value `0x8` appears when the supply voltage reading
  falls to about 15 V (seen when damping brakes from top speed upset a 24 V bench supply). Latched, it
  blocks run mode and `enable()` raises `MotorFaultError`; `restart()` or a power cycle clears it. Other
  bits are unknown. Stop fast joints with `disable()` (idle), not with a damping command.
- **Position reference.** After a power cycle or `restart()` the position reading can come back shifted by
  whole output turns (524,288 counts on one wrist motor, 819,200 on another). Re-measure end stops each time.
- **Other modes.** 3, 4 and 5 are accepted but ignore all command fields: 3 and 5 brake the joint like idle,
  4 lets it free-wheel. 2 and 6 are ignored. Idle (mode 0) itself brakes the joint; it is not free.
- **Rate.** About 340 exchanges per second per motor with three motors from Python; the robot's
  controller does about 1,300.

Commanding a motor moves real hardware. Keep clear of the joints, start with low gains, and set
limits with `set_limits`.

## Tests

    pytest tests        # frame encode/decode against captured frames; no hardware needed

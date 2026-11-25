# g1-joint-motor-sdk

Unofficial Python driver for Unitree G1 joint motors, talking to them directly over their
RS-485 bus. Reverse engineered from bus captures on 2026-11-25; not affiliated with or
supported by Unitree. Version 0.1.0, Linux only.

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
# leaving the block brakes the motors with damping, then puts them in idle
```

Command line:

    g1motor scan
    g1motor read --ids 4,5,6 --seconds 10 [--zero-torque]
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
| `bus.disable(ids=None)` | Damping brake for 0.5 s, then idle. Also done on leaving the `with` block. |
| `bus.listen(seconds)` | Raw bytes, no transmission. |
| `MotorCmd`, `MotorState` | Frame encode / decode, also usable on captured data. |
| `units` | Counts to rad, rad/s, N*m. **Assumed scales**, rotor side. |

The motor applies `tau + kp * (pos - measured) + kd * (vel - measured)` and keeps applying the
last command it received. Call `command()` for every enabled motor in a steady loop. What a
motor does when polling simply stops (no brake, no idle) has not been tested.

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
| 3 | 1 | temperature A (deg C, likely) |
| 4 | 1 | temperature B |
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
- Scan, zero torque, position control with kp/kd, velocity and torque feed-forward: on a bench
  wrist (ids 4, 5, 6) at 24 V. Full-range sines up to a 0.8 s period.
- Those motors top out at about 1,275,000 position counts/s on 24 V.

Not verified, or known gaps:
- **Physical units.** `units` uses GO-M8010-6 scales; only the position/velocity ratio is checked.
  Gear ratios are unknown, so everything is rotor side.
- **Gain scale.** kp and kd are raw; tuned values for the bench wrist were kp 300, kd 40-80.
- **Other joints.** Only wrist motors were driven. On the robot, two arm motors (ids 2 and 3 of
  the chain observed) need a 12-byte payload at boot before they report telemetry. This SDK does
  not send it, and it is unknown whether that payload is fixed or a challenge and response.
- **Byte 3 and the flag bit.** Reproduced as the controller sends them; whether they are required
  is untested.
- **Faults.** `MotorState.error` is the status word. Value `0x8` latched on all three bench motors
  after a hard stop from top speed on a 24 V bench supply (cause not confirmed), and they then refused
  run mode (whether a power cycle clears it is not yet tested); `enable()` raises `MotorFaultError` in that case. The meaning of
  individual bits and any way to clear a fault over the bus are unknown.
- **Rate.** About 340 exchanges per second per motor with three motors from Python; the robot's
  controller does about 1,300.

Commanding a motor moves real hardware. Keep clear of the joints, start with low gains, and set
limits with `set_limits`.

## Tests

    pytest tests        # frame encode/decode against captured frames; no hardware needed

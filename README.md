# Unitree motor reverse engineering

Working notes, captures and tools for reverse engineering a Unitree actuator.

## Target

- Robot: Unitree G1, one arm chain (5 motors, ids 0-4)
- Firmware version: _TBD_
- Decoded protocol: `protocol/g1-arm-bus.md`
- Manual (PDF): `docs/G1_Joint_Motor_Protocol_Guide.pdf` and Chinese `docs/G1_Joint_Motor_Protocol_Guide_zh.pdf`; sources `docs/manual/manual.html` and `docs/manual/make_zh.py`; rebuild both with `docs/manual/build.sh`
- Driver package: `sdk/` (`g1_joint_motor_sdk`, command `g1motor`)
- Interface: RS-485 at 6,000,000 baud 8N1, FT232H adapter (serial FT866L1W)

## Layout

| Folder      | Contents                                                        |
|-------------|-----------------------------------------------------------------|
| `docs/`     | Datasheets, vendor SDK references, teardown photos, notes       |
| `captures/` | Raw bus captures and logic-analyzer traces (one folder per session) |
| `firmware/` | Firmware dumps and updater images (not tracked by git)          |
| `protocol/` | Decoded packet layouts, CRC, command tables                     |
| `scripts/`  | Python tools for talking to the motor and parsing captures      |
| `analysis/` | MATLAB scripts for parameter and system identification          |

## Log

Record each session here: date, setup, what was learned.

# Unitree motor reverse engineering

Working notes, captures and tools for reverse engineering a Unitree actuator.

## Target

- Robot: Unitree G1, one arm chain (5 motors, ids 0-4)
- Firmware version: _TBD_
- Decoded protocol: `protocol/g1-arm-bus.md`
- Manual (PDF): `docs/G1_Joint_Motor_Protocol_Guide.pdf` and Chinese `docs/G1_Joint_Motor_Protocol_Guide_zh.pdf`; sources `docs/manual/manual.html` and `docs/manual/make_zh.py`; rebuild both with `docs/manual/build.sh`
- Drivers: `sdk/python` (`g1_joint_motor_sdk`, command `g1motor`), `sdk/cpp`, `sdk/matlab`
- Interface: RS-485 at 6,000,000 baud 8N1, FT232H adapter (serial FT866L1W)

## Layout

| Folder | Contents |
|---|---|
| `protocol/` | Working notes: decoded layout, boot sequence, every test and its numbers |
| `docs/` | The protocol guide as PDF (English and Chinese) and its sources in `docs/manual/` |
| `sdk/` | Drivers with the same API in Python (`sdk/python`), C++ (`sdk/cpp`) and MATLAB (`sdk/matlab`); see `sdk/README.md` |
| `scripts/` | Research tools: `sniff.py` (capture), `monitor.py` (live plots), `sweep.py` (motion tests), `mode_*.py`, `fault_clear_test.py` |
| `calibration/` | End stops (`limits.json`, re-measure after power-on) and the friction feed-forward table |
| `captures/` | Raw bus captures and motion logs, one folder per day (not tracked by git) |
| `firmware/`, `analysis/` | Empty so far |

## Log

Record each session here: date, setup, what was learned.

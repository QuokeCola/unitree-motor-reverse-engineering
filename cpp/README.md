# g1motor for C++

C++17, no dependencies.

Read the repository’s [wiring and bench setup](../README.md#wiring) first.
Commands below assume your working directory is `cpp/`.

## Build

Requires CMake 3.16+ and a C++17 compiler.

```sh
cmake -S . -B build
cmake --build build
```

## Find motors

```sh
./build/scan
./build/scan /dev/ttyUSB0  # explicit port
```

With no port, the driver selects the only FTDI adapter. A scan sends idle frames;
run it before enabling motors, with the robot controller disconnected.

## Hold one motor

This example assumes motor ID 4 was found by a scan. Change the ID to match your
bus. It enables and polls only that motor. The gains were tested on a 24 V bench
wrist; support the joint before enabling it.

```cpp
#include "g1motor/g1motor.h"

g1motor::MotorBus bus;                           // the only FTDI adapter, 6 Mbaud
auto start = bus.enable({4});              // run mode, zero torque: joints are free
for (int i = 0; i < 1000; ++i) {                 // hold joint 4 where it is
  auto state = bus.command(4, start[4].pos, 0, 0, 300, 40);
}
// the destructor returns the motors to idle, which brakes them
```

## API

Everything is in the namespace `g1motor` (files `protocol`, `serial_port`, `motor_bus`, `joint`,
`units` in `g1motor/`). The names are the same in Python and MATLAB.

| Name | What it does |
|---|---|
| `MotorBus(port, reply_timeout, check_silent)` | Opens the adapter. Raises `BusBusyError` if the bus is not silent. |
| `bus.scan(tries)` | Polls ids 0-14 with the idle frame; returns the motors found, by id. |
| `bus.enable(ids)` | Run mode with zero torque. Raises `MotorFaultError` if a motor refuses. |
| `bus.command(id, pos, vel, tau, kp, kd)` | One hybrid command in raw counts; returns the fresh state, or no reply on timeout / invalid reply. |
| `bus.read(id)` | State only. On an enabled motor this sends zero torque, which drops a hold. |
| `bus.disable(ids)` | Back to idle, which brakes the joint. No ids = every enabled motor. |
| `bus.restart(ids, wait)` | Mode 7: clears a latched fault; about 0.3 s. Positions may come back shifted. |
| `bus.set_joint(id, joint)`, `bus.joint(id)`, `bus.is_enabled(id)` | Attach and read back a `Joint`; position targets are clamped to its end stops when `kp` is nonzero. |
| `bus.transfer(cmd)`, `bus.listen(seconds)`, `bus.close()` | One raw exchange; receive without sending; back to idle and close. |
| `MotorCmd`: `encode()`, `decode(frame)` | Command frame. Fields `id mode tau vel pos kp kd flag byte3`. |
| `MotorState`: `decode(frame)`, `voltage()`, `fault()` | Reply frame. Fields `id mode tau vel pos temp_coil temp_shell voltage_raw error word18 byte20 byte21`. |
| `Joint(gear_ratio, low, high)`: `turn_counts()`, `travel()`, `sync(pos)`, `angle(pos)`, `counts(angle)`, `speed(vel)`, `clamp(pos)` | Gear ratio, end stops, software zero, output-shaft angles. |
| `load_joints(path, gear_ratios, margin)` | Joints from an end-stop file. |
| `crc32_core(data)`, `find_reply(buffer)` | Checksum; first valid reply in a buffer. |
| `units`: `pos_to_rad`, `rad_to_pos`, `vel_to_rads`, `rads_to_vel`, `pos_rate_to_vel`, `tau_to_nm`, `nm_to_tau` | Rotor-side conversions (torque scale assumed). |
| `BAUD`, `CMD_LEN`, `REPLY_LEN`, `BROADCAST_ID`, `MODE_IDLE`, `MODE_RUN`, `MODE_RESTART`, `COUNTS_PER_ROTOR_TURN`, `WRIST_GEAR_RATIOS` | Constants. |

The motor applies `tau + kp * (pos - measured) + kd * (vel - measured)` and keeps applying the last
command. These fields use protocol scaling; the expression describes the control
terms, not a formula to evaluate directly with raw counts. `pos` and `vel` are
rotor-side targets; torque conversion to N·m is assumed rather than measured.
Use `units` for rotor conversions and an attached `Joint` for output-shaft angles.

Call `command` for every enabled motor in a steady loop; a motor that is no longer polled
drops to idle after 0.5-1 s. End stops in counts must be re-checked after every power-on, because
the position reading can shift by whole output turns.

C++ only: `SerialPort`. Errors are exceptions; `decode`, `read`, `command` and `transfer` return
`std::optional`.

## Tests

    ctest --test-dir build              # protocol_test and joint_test, no hardware
    build/hardware_test                 # motors on a bus without a controller

## Examples

`examples/scan.cc`, `zero_torque.cc`, `position_hold.cc`, `sine.cc`, built into `build/`.

Only the Linux serial code has been compiled and run.

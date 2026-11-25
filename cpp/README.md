# C++ interface

`g1motor` static library, C++17, no dependencies.

    cmake -S . -B build && cmake --build build
    build/g1motor_scan                  # optional argument: the port name

```cpp
#include "g1motor/g1motor.h"

g1motor::MotorBus bus;                           // the only FTDI adapter (Linux)
auto start = bus.Enable({4, 5, 6});              // run mode, zero torque
for (int i = 0; i < 1000; ++i) {
  auto state = bus.Command(4, start[4].pos, 0, 0, 300, 40);   // hold joint 4
}
// the destructor returns the motors to idle, which brakes them
```

| Header | Contents |
|---|---|
| `g1motor/protocol.h` | `MotorCmd`, `MotorState`, `Crc32Core`, `FindReply` |
| `g1motor/serial_port.h` | `SerialPort`: 6 Mbaud on Linux, macOS, Windows |
| `g1motor/motor_bus.h` | `MotorBus`: `Scan`, `Enable`, `Command`, `Read`, `Disable`, `Restart` |
| `g1motor/joint.h` | `Joint`: gear ratio, end stops, software zero |

Errors are reported with exceptions (`BusBusyError`, `MotorFaultError`, `std::runtime_error`).

## Tests and examples

    ctest --test-dir build              # g1motor_protocol_test, no hardware
    build/g1motor_hardware_test         # motors on a bus without a controller

Examples: `examples/scan.cc`, `examples/position_hold.cc`.

Only the Linux serial code has been compiled and run.

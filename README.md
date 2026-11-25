# G1 joint motor SDK

Unofficial drivers for the joint motors of the Unitree G1 on their RS-485 bus, in Python, C++
and MATLAB, with a description of the communication protocol. Reverse engineered from bus
captures and bench tests; not affiliated with or supported by Unitree.

Use the drivers on a bare arm or motor with no robot controller on the bus. On a complete robot
the onboard controller is the bus master; the drivers refuse to open a bus that has traffic.

## Layout

| Folder | Contents |
|---|---|
| `docs/` | The communication protocol: `G1_Joint_Motor_Protocol_Guide.pdf` (English) and `_zh.pdf` (Chinese), their sources, lab notes, and the Doxygen configuration |
| `python/` | `g1_joint_motor_sdk` package, the `g1motor` command, tests, examples, and the research tools in `python/tools` |
| `cpp/` | `g1motor` C++17 library (CMake, no dependencies), tests, examples |
| `matlab/` | `G1MotorBus` class over a MEX build of the C++ library, tests, example |

The three interfaces have the same calls:

| | Python | C++ | MATLAB |
|---|---|---|---|
| Open | `bus = MotorBus()` | `g1motor::MotorBus bus;` | `bus = G1MotorBus();` |
| Find motors | `bus.scan()` | `bus.Scan()` | `bus.scan()` |
| Run mode, zero torque | `bus.enable([4, 5, 6])` | `bus.Enable({4, 5, 6})` | `bus.enable([4 5 6])` |
| Command | `bus.command(4, pos=p, kp=300, kd=40)` | `bus.Command(4, p, 0, 0, 300, 40)` | `bus.command(4, 'pos', p, 'kp', 300, 'kd', 40)` |
| Stop (idle brakes the joint) | `bus.disable()` | `bus.Disable()` | `bus.disable()` |
| Clear a fault | `bus.restart([4])` | `bus.Restart({4})` | `bus.restart(4)` |

Values are raw counts. See each folder's README for building, examples and tests.

## Tests

Each language has a protocol test that needs no hardware and a hardware test for motors on a
bus without a controller (it holds the joints in place for 2 s and restarts one motor).

| | Protocol test | Hardware test |
|---|---|---|
| Python | `python -m pytest python` | `python python/tests/hardware_test.py` |
| C++ | `ctest --test-dir cpp/build` | `cpp/build/g1motor_hardware_test` |
| MATLAB | `protocol_test` (in `matlab/tests`) | `hardware_test` (in `matlab/tests`) |

## Documentation

- **Protocol:** `docs/G1_Joint_Motor_Protocol_Guide.pdf`. Rebuild with `docs/manual/build.sh`.
- **API reference:** every class and function carries Doxygen comments. Build it with
  `doxygen docs/Doxyfile` from the top of the repository; open `docs/api/html/index.html`.
  MATLAB files are read through `docs/doxygen_matlab_filter.py`.

## Code style

Names follow the Google style guides: C++ files are `snake_case.h` / `.cc` with `PascalCase`
types and functions, `snake_case` variables, a trailing underscore on class members and
`kPascalCase` constants; Python uses `snake_case` modules and functions, `PascalCase` classes and
`UPPER_CASE` constants. C++ is formatted with `clang-format` (Google style, `cpp/.clang-format`).
Google has no MATLAB guide; MATLAB uses a `PascalCase` class with `lowerCamelCase` methods and
`snake_case` function files. The C++ library uses exceptions, which the Google C++ guide does not.

## Serial port

The bus runs at 6,000,000 baud, a non-standard rate. No general serial library is used for C++;
`cpp/src/serial_port.cc` calls each operating system's own interface.

| System | Method | Status |
|---|---|---|
| Linux | `termios2` with `BOTHER` (`TCSETS2`), FTDI low-latency flag | run on hardware in all three languages |
| Windows | Win32 `CreateFile` / `DCB.BaudRate` | written, never compiled or run |
| macOS | `termios` + `IOSSIOSPEED` | written, never compiled or run |

- Python uses `pyserial`; on Linux the rate is set with the same `TCSETS2` call because pyserial's
  own custom-rate path fails on glibc 2.42 and later ([pyserial issue 805](https://github.com/pyserial/pyserial/issues/805)).
- Apple's built-in FTDI driver is reported to stop at about 3 Mbaud
  ([Apple forum](https://developer.apple.com/forums/thread/771291)). If macOS refuses 6 Mbaud, the
  options are FTDI's own VCP driver or a backend that talks to the chip directly (FTDI D2XX,
  libftdi, pyftdi); none is implemented here.
- An FTDI FT232H RS-485 adapter is needed (the Unitree one works). Its latency timer limits the
  polling rate: set to 1 ms automatically on Linux, by hand on Windows (Device Manager, the COM
  port, Port Settings, Advanced). At 1 ms all three interfaces reach about 330 exchanges per
  second per motor with three motors; the robot's own controller does about 1,313.

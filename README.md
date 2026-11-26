# g1motor

Unofficial drivers for the joint motors of the Unitree G1 on their RS-485 bus, in Python, C++
and MATLAB, with a description of the communication protocol. Reverse engineered from bus
captures and bench tests; not affiliated with or supported by Unitree.

Use the drivers on a bare arm or motor with no robot controller on the bus. On a complete robot
the onboard controller is the bus master; the drivers refuse to open a bus that has traffic.

## Layout

| Folder | Contents |
|---|---|
| `docs/` | The communication protocol: `G1_Joint_Motor_Protocol_Guide.pdf` (English) and `_zh.pdf` (Chinese), their sources, lab notes, and the Doxygen configuration |
| `python/` | `g1motor` for Python |
| `cpp/` | `g1motor` for C++17 |
| `matlab/` | `g1motor` for MATLAB (calls the C++ library through a MEX file) |

Each language folder has the same four parts:

| Part | Python | C++ | MATLAB |
|---|---|---|---|
| Library | `python/g1motor/` | `cpp/g1motor/` | `matlab/+g1motor/` |
| Tests | `python/tests/` | `cpp/tests/` | `matlab/tests/` |
| Examples | `python/examples/` | `cpp/examples/` | `matlab/examples/` |
| Build | `pyproject.toml` | `CMakeLists.txt` | `build_mex.m` |

and the same names inside: the library is split into `protocol`, `serial_port`, `motor_bus`,
`joint` and `units`; the tests are `protocol_test`, `joint_test` and `hardware_test`; the examples
are `scan`, `zero_torque`, `position_hold` and `sine`. (`python/tools` additionally holds the
research scripts used to reverse engineer the bus.)

## One API in three languages

| | Python | C++ | MATLAB |
|---|---|---|---|
| Open | `bus = g1motor.MotorBus()` | `g1motor::MotorBus bus;` | `bus = g1motor.MotorBus();` |
| Find motors | `bus.scan()` | `bus.scan()` | `bus.scan()` |
| Run mode, zero torque | `bus.enable([4, 5, 6])` | `bus.enable({4, 5, 6})` | `bus.enable([4 5 6])` |
| Command | `bus.command(4, pos, 0, 0, 300, 40)` | `bus.command(4, pos, 0, 0, 300, 40)` | `bus.command(4, pos, 0, 0, 300, 40)` |
| Stop (idle brakes the joint) | `bus.disable()` | `bus.disable()` | `bus.disable()` |
| Clear a fault | `bus.restart([4])` | `bus.restart({4})` | `bus.restart(4)` |
| Joint angle | `bus.joint(4).angle(pos)` | `bus.joint(4).angle(pos)` | `bus.joint(4).angle(pos)` |
| Checksum | `g1motor.crc32_core(data)` | `g1motor::crc32_core(data, len)` | `g1motor.crc32_core(data)` |
| Constant | `g1motor.BAUD` | `g1motor::BAUD` | `g1motor.BAUD` |

Values are raw counts. The full list of names is in each folder's README.

## Naming

One rule in all three languages: functions, methods and variables are `snake_case`, types are
`PascalCase`, constants are `UPPER_CASE`, files are `snake_case` (a MATLAB class file is named after
its class). This is the Google Python style. C++ follows the Google C++ guide in everything else
(include guards, trailing underscore on class members, `clang-format`); its function and constant
names follow the common rule instead, so that the three interfaces read the same.

## Tests

Each language has two tests that need no hardware and one that needs motors on a bus without a
controller (it holds the joints in place for 2 s and restarts one motor).

| | `protocol_test`, `joint_test` | `hardware_test` |
|---|---|---|
| Python | `python -m pytest python` | `python python/tests/hardware_test.py` |
| C++ | `ctest --test-dir cpp/build` | `cpp/build/hardware_test` |
| MATLAB | `protocol_test`, `joint_test` (in `matlab/tests`) | `hardware_test` (in `matlab/tests`) |

## Documentation

- **Protocol:** `docs/G1_Joint_Motor_Protocol_Guide.pdf`. Rebuild with `docs/manual/build.sh`.
- **API reference:** every class and function carries Doxygen comments. Build it with
  `doxygen docs/Doxyfile` from the top of the repository; open `docs/api/html/index.html`.
  MATLAB files are read through `docs/doxygen_matlab_filter.py`.

## Serial port

The bus runs at 6,000,000 baud, a non-standard rate. No general serial library is used for C++;
`cpp/g1motor/serial_port.cc` calls each operating system's own interface.

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

## License

MIT; see [`LICENSE`](LICENSE). Unitree and G1 are trademarks of Unitree Robotics, which neither
endorses nor supports this project.

# G1 joint motor SDK

Unofficial drivers for Unitree G1 joint motors on their RS-485 bus, in three languages with the
same API. Reverse engineered; not affiliated with or supported by Unitree. The protocol is
described in `docs/G1_Joint_Motor_Protocol_Guide.pdf`.

| Folder | Language | What it is |
|---|---|---|
| `python/` | Python 3.10+ | `g1_joint_motor_sdk` package and the `g1motor` command; needs `pyserial` |
| `cpp/` | C++17 | `g1motor` static library, CMake, no dependencies |
| `matlab/` | MATLAB R2018a+ | `G1MotorBus` class over a MEX build of the C++ library |

Use them on a bare arm or motor with no robot controller on the bus. All three refuse to open a
bus that already has traffic on it.

## The same five calls everywhere

| | Python | C++ | MATLAB |
|---|---|---|---|
| Open | `bus = MotorBus()` | `g1motor::MotorBus bus;` | `bus = G1MotorBus();` |
| Find motors | `bus.scan()` | `bus.scan()` | `bus.scan()` |
| Run mode, zero torque | `bus.enable([4, 5, 6])` | `bus.enable({4, 5, 6})` | `bus.enable([4 5 6])` |
| Command | `bus.command(4, pos=p, kp=300, kd=40)` | `bus.command(4, p, 0, 0, 300, 40)` | `bus.command(4, 'pos', p, 'kp', 300, 'kd', 40)` |
| Stop (idle brakes the joint) | `bus.disable()` | `bus.disable()` | `bus.disable()` |
| Clear a fault | `bus.restart([4])` | `bus.restart({4})` | `bus.restart(4)` |

Values are raw counts. `Joint` (Python, C++) and `setJoint` / `jointAngle` (MATLAB) add gear ratio,
end stops and output-shaft angles. Details: `python/README.md`.

## Build and run

Python:

    pip install -e python
    g1motor scan

C++:

    cmake -S cpp -B cpp/build && cmake --build cpp/build
    ctest --test-dir cpp/build          # frame tests, no hardware
    cpp/build/g1motor_scan              # optional argument: the port name

MATLAB (needs a C++17 compiler set up with `mex -setup C++`):

    cd matlab
    build_mex
    bus = G1MotorBus();  struct2table(bus.scan())

## Serial port: what is used and why

The bus runs at 6,000,000 baud, which is not a standard rate. No general serial library was
adopted for C++; `cpp/src/serial_port.cpp` calls each operating system's own interface, about
200 lines in total:

| System | Method | Status |
|---|---|---|
| Linux | `termios2` with `BOTHER` (`TCSETS2`), FTDI low-latency flag | run on hardware in all three languages |
| Windows | Win32 `CreateFile` / `DCB.BaudRate` | written, not run |
| macOS | `termios` + `IOSSIOSPEED` | written, not run |

Reasons, from a search on 2026-11-25:

- **Linux custom rates are fragile in libraries.** glibc 2.42 changed its termios handling, which broke
  pyserial's custom-rate path (EINVAL; pyserial issue 805, seen on this machine) and required a fix in
  libserialport. A forum report also describes libserialport accepting a non-standard rate without
  applying it. The `TCSETS2` ioctl itself is stable.
- **Windows** takes any rate through the DCB, so no library is needed there.
- **macOS** takes custom rates through `IOSSIOSPEED`, but Apple's built-in FTDI driver is reported
  to stop at about 3 Mbaud. If 6 Mbaud is refused there, the options are FTDI's own VCP driver, or
  talking to the chip directly with FTDI's D2XX library, libftdi, or pyftdi (pure Python). None of
  those is implemented here; they would be a second backend behind `SerialPort`.
- **Python** keeps `pyserial` for opening the port and reading, with the Linux rate set by the same
  `TCSETS2` call; on Windows and macOS pyserial sets the rate itself.
- **MATLAB** goes through the C++ code (MEX) instead of MATLAB's `serialport`, whose documentation
  does not say whether non-standard rates are supported on each system.

Sources: [pyserial issue 805](https://github.com/pyserial/pyserial/issues/805),
[Arch forum thread on libserialport](https://bbs.archlinux.org/viewtopic.php?id=308199%2F),
[Apple forum: serial speed limited to 3 Mbps](https://developer.apple.com/forums/thread/771291),
[pySerial API notes on custom rates](https://pyserial.readthedocs.io/en/stable/pyserial_api.html),
[FTDI AN232B-05 baud rates](https://www.ftdichip.com/old2020/Support/Documents/AppNotes/AN232B-05_BaudRates.pdf),
[FTDI AN_107 driver options (latency timer)](https://www.ftdichip.com/old2020/support/Documents/AppNotes/AN_107_AdvancedDriverOptions_AN_000073.pdf),
[MathWorks serialport](https://www.mathworks.com/help/matlab/ref/serialport.html).

## Platform notes

- **Adapter.** An FTDI FT232H RS-485 adapter (the Unitree one works). Other USB serial chips may not
  reach 6 Mbaud.
- **Latency timer.** FTDI adapters hold short replies for a latency time, 16 ms by default. On Linux
  the SDK sets it to 1 ms. On Windows set it by hand: Device Manager, the COM port, Port Settings,
  Advanced, Latency Timer = 1 ms. At 16 ms the polling rate would be about 60 exchanges per second.
- **Polling rate.** With the 1 ms latency timer, one exchange takes about 1 ms in every language:
  about 330 per second per motor with three motors (Python 332, C++ 334, MATLAB 328 measured). The
  robot's controller does about 1,313 per motor. Going faster needs the latency timer below 1 ms,
  which means talking to the FTDI chip directly.
- **Permissions (Linux).** The user must be in the group that owns the serial device (`uucp` or `dialout`).
- **Port names.** Linux finds the only FTDI adapter by itself. On Windows and macOS give the name
  (`COM5`, `/dev/cu.usbserial-XXXX`) in C++ and MATLAB; Python looks for an FTDI device on all three.

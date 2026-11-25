"""!
@file serial_port.py
@brief Serial port at 6 Mbaud on Linux, Windows and macOS.

Linux: pyserial opens the port and the rate is set with TCSETS2 / BOTHER,
because pyserial's own custom-rate path fails on glibc 2.42 and later.
Windows and macOS: pyserial sets the rate itself. Only the Linux path has
been run on hardware.
"""
from __future__ import annotations

import glob
import struct
import sys
import time

import serial
from serial.tools import list_ports

from g1_joint_motor_sdk import protocol

## USB vendor id of FTDI.
FTDI_VID = 0x0403

## Linux ioctl that reads struct termios2.
_TCGETS2 = 0x802C542A
## Linux ioctl that writes struct termios2.
_TCSETS2 = 0x402C542B
## c_cflag value selecting the rate given in c_ispeed / c_ospeed.
_BOTHER = 0o010000
## Mask of the rate bits in c_cflag.
_CBAUD = 0o010017
## Layout of struct termios2 for the struct module.
_TERMIOS2_FORMAT = "4I20s2I"


def find_port() -> str:
    """!
    @brief Finds the FTDI adapter when exactly one is plugged in.
    @return The device name.
    @exception RuntimeError None or several adapters were found.
    """
    ports = []
    if sys.platform.startswith("linux"):
        ports = sorted(glob.glob("/dev/serial/by-id/*FTDI*"))
    if not ports:
        ports = sorted(port.device for port in list_ports.comports()
                       if port.vid == FTDI_VID)
    if len(ports) == 1:
        return ports[0]
    if not ports:
        raise RuntimeError("no FTDI serial adapter found; pass port=")
    raise RuntimeError("several FTDI adapters found, pass port=:\n  "
                       + "\n  ".join(ports))


def set_baud(fd: int, baud: int) -> int:
    """!
    @brief Sets any bit rate on Linux with TCSETS2 / BOTHER.
    @param fd File descriptor of the open port.
    @param baud Bit rate.
    @return The rate the driver reports back.
    """
    import fcntl  # Linux only.

    raw = bytearray(struct.calcsize(_TERMIOS2_FORMAT))
    fcntl.ioctl(fd, _TCGETS2, raw)
    fields = list(struct.unpack(_TERMIOS2_FORMAT, bytes(raw)))
    fields[2] = (fields[2] & ~_CBAUD) | _BOTHER
    fields[5] = fields[6] = baud
    fcntl.ioctl(fd, _TCSETS2, struct.pack(_TERMIOS2_FORMAT, *fields))
    fcntl.ioctl(fd, _TCGETS2, raw)
    return struct.unpack(_TERMIOS2_FORMAT, bytes(raw))[5]


def open_port(port: str | None = None,
              baud: int = protocol.BAUD) -> serial.Serial:
    """!
    @brief Opens the port in raw 8N1 mode at the given rate.

    On Linux the FTDI latency timer is set to 1 ms. On Windows set it by
    hand in Device Manager, or the polling rate drops to about 60 per second.

    @param port Device name; None selects the only FTDI adapter.
    @param baud Bit rate.
    @return The open pyserial port.
    @exception RuntimeError The port cannot be opened or refuses the rate.
    """
    port = port or find_port()
    if sys.platform.startswith("linux"):
        ser = serial.Serial(port, 4_000_000, timeout=0.0005)
        actual = set_baud(ser.fd, baud)
        if actual != baud:
            ser.close()
            raise RuntimeError(f"asked for {baud} baud, driver reports "
                               f"{actual}")
        try:
            ser.set_low_latency_mode(True)
        except (OSError, ValueError, NotImplementedError):
            pass  # Not an FTDI adapter, or not permitted.
    else:
        try:
            ser = serial.Serial(port, baud, timeout=0.0005)
        except (OSError, ValueError) as error:
            hint = ""
            if sys.platform == "darwin":
                hint = (" Apple's built-in FTDI driver is reported to stop "
                        "at about 3 Mbaud; try FTDI's VCP driver.")
            raise RuntimeError(f"cannot open {port} at {baud} baud: "
                               f"{error}.{hint}") from None
    time.sleep(0.1)  # Let bytes received before the rate change drain.
    ser.reset_input_buffer()
    return ser

"""Serial port at 6 Mbaud, 8N1, on Linux, Windows and macOS.

  Linux    pyserial opens the port, then the rate is set with TCSETS2 / BOTHER. pyserial's own
           custom-rate path fails with EINVAL on glibc 2.42 and later (pyserial issue 805).
  Windows  pyserial passes the rate to the driver through the Win32 DCB.
  macOS    pyserial sets it with the IOSSIOSPEED ioctl. Apple's built-in FTDI driver is reported to
           stop at about 3 Mbaud, so 6 Mbaud may need FTDI's own VCP driver.
Only the Linux path has been run on hardware.
"""
from __future__ import annotations

import glob
import struct
import sys
import time

import serial
from serial.tools import list_ports

from .protocol import BAUD

FTDI_VID = 0x0403


def find_port() -> str:
    """The FTDI adapter, if exactly one is plugged in."""
    ports = sorted(glob.glob("/dev/serial/by-id/*FTDI*")) if sys.platform.startswith("linux") else []
    if not ports:
        ports = sorted(p.device for p in list_ports.comports() if p.vid == FTDI_VID)
    if len(ports) == 1:
        return ports[0]
    if not ports:
        raise RuntimeError("no FTDI serial adapter found; pass port= explicitly")
    raise RuntimeError("several FTDI adapters found, pass port= explicitly:\n  " + "\n  ".join(ports))


def set_baud(fd: int, baud: int) -> int:
    """Linux: set any baud rate with TCSETS2 / BOTHER; returns the rate the driver reports back."""
    import fcntl
    tcgets2, tcsets2, bother, cbaud = 0x802C542A, 0x402C542B, 0o010000, 0o010017
    raw = bytearray(44)
    fcntl.ioctl(fd, tcgets2, raw)
    fields = list(struct.unpack("4I20s2I", bytes(raw)))
    fields[2] = (fields[2] & ~cbaud) | bother
    fields[5] = fields[6] = baud
    fcntl.ioctl(fd, tcsets2, struct.pack("4I20s2I", *fields))
    fcntl.ioctl(fd, tcgets2, raw)
    return struct.unpack("4I20s2I", bytes(raw))[5]


def open_port(port: str | None = None, baud: int = BAUD) -> serial.Serial:
    port = port or find_port()
    if sys.platform.startswith("linux"):
        ser = serial.Serial(port, 4_000_000, bytesize=8, parity="N", stopbits=1, timeout=0.0005)
        actual = set_baud(ser.fd, baud)
        if actual != baud:
            ser.close()
            raise RuntimeError(f"asked for {baud} baud, driver reports {actual}")
        try:
            ser.set_low_latency_mode(True)   # FTDI latency timer 16 ms -> 1 ms
        except Exception:  # noqa: BLE001  (not an FTDI adapter, or not permitted)
            pass
    else:
        try:
            ser = serial.Serial(port, baud, bytesize=8, parity="N", stopbits=1, timeout=0.0005)
        except (OSError, ValueError, serial.SerialException) as e:
            hint = (" Apple's built-in FTDI driver is reported to stop at about 3 Mbaud; try FTDI's VCP driver."
                    if sys.platform == "darwin" else "")
            raise RuntimeError(f"cannot open {port} at {baud} baud: {e}.{hint}") from None
        # The FTDI latency timer (16 ms by default) limits the polling rate. Windows: Device Manager,
        # the COM port, Port Settings, Advanced, Latency Timer = 1 ms.
    time.sleep(0.1)                          # let bytes received before the rate change drain
    ser.reset_input_buffer()
    return ser

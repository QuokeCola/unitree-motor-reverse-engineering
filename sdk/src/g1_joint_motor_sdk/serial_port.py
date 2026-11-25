"""Serial port at 6 Mbaud (Linux). pyserial cannot set this rate on every system, so it is
set with the TCSETS2 / BOTHER ioctl after opening at a standard rate."""
from __future__ import annotations

import fcntl
import glob
import struct
import time

import serial

from .protocol import BAUD

_TCGETS2, _TCSETS2, _BOTHER, _CBAUD = 0x802C542A, 0x402C542B, 0o010000, 0o010017


def find_port() -> str:
    """The FTDI adapter, if exactly one is plugged in."""
    ports = sorted(glob.glob("/dev/serial/by-id/*FTDI*"))
    if len(ports) == 1:
        return ports[0]
    if not ports:
        raise RuntimeError("no FTDI serial adapter found; pass port= explicitly")
    raise RuntimeError("several FTDI adapters found, pass port= explicitly:\n  " + "\n  ".join(ports))


def set_baud(fd: int, baud: int) -> int:
    """Set any baud rate; returns the rate the driver reports back."""
    raw = bytearray(44)
    fcntl.ioctl(fd, _TCGETS2, raw)
    fields = list(struct.unpack("4I20s2I", bytes(raw)))
    fields[2] = (fields[2] & ~_CBAUD) | _BOTHER
    fields[5] = fields[6] = baud
    fcntl.ioctl(fd, _TCSETS2, struct.pack("4I20s2I", *fields))
    fcntl.ioctl(fd, _TCGETS2, raw)
    return struct.unpack("4I20s2I", bytes(raw))[5]


def open_port(port: str | None = None, baud: int = BAUD) -> serial.Serial:
    ser = serial.Serial(port or find_port(), 4_000_000, bytesize=8, parity="N", stopbits=1, timeout=0.0005)
    actual = set_baud(ser.fd, baud)
    if actual != baud:
        ser.close()
        raise RuntimeError(f"asked for {baud} baud, driver reports {actual}")
    try:
        ser.set_low_latency_mode(True)   # FTDI latency timer 16 ms -> 1 ms
    except Exception:  # noqa: BLE001  (not an FTDI adapter, or not permitted)
        pass
    time.sleep(0.1)                      # let bytes received before the rate change drain
    ser.reset_input_buffer()
    return ser

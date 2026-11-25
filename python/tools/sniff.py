#!/usr/bin/env python3
"""Passive RS-485 bus capture. Never transmits.

Writes two files per run into captures/<session>/:
  <name>.bin  raw bytes exactly as received
  <name>.idx  one line per read: "<seconds since start> <offset in .bin> <length>"

Usage:
  sniff.py [--baud 6000000] [--seconds 120] [--name poweron]
"""
import argparse
import fcntl
import os
import struct
import sys
import time
from datetime import datetime

import serial

PORT = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT866L1W-if00-port0"
ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "captures")

# Frame headers used by the public Unitree motor SDKs (A1/B1/GO-M8010-6).
HEADERS = {
    "FE EE (host -> motor)": b"\xfe\xee",
    "FD EE (motor -> host, public SDK)": b"\xfd\xee",
    "FC EE (seen on G1 arm bus)": b"\xfc\xee",
}
BURST_GAP_S = 0.005  # reads closer together than this are shown as one burst


def set_baud(fd: int, baud: int) -> int:
    """Set an arbitrary baud rate via termios2/BOTHER (pyserial fails on 6M here)."""
    TCGETS2, TCSETS2, BOTHER, CBAUD = 0x802C542A, 0x402C542B, 0o010000, 0o010017
    raw = bytearray(44)
    fcntl.ioctl(fd, TCGETS2, raw)
    f = list(struct.unpack("4I20s2I", bytes(raw)))
    f[2] = (f[2] & ~CBAUD) | BOTHER
    f[5] = f[6] = baud
    fcntl.ioctl(fd, TCSETS2, struct.pack("4I20s2I", *f))
    fcntl.ioctl(fd, TCGETS2, raw)
    return struct.unpack("4I20s2I", bytes(raw))[5]


def summarize(data: bytes, index: list, baud: int) -> None:
    print(f"\n--- summary @ {baud} baud ---")
    print(f"total bytes: {len(data)}")
    if not data:
        print("bus was silent for the whole capture")
        return
    print(f"first byte at t = {index[0][0]:.4f} s, last read at t = {index[-1][0]:.4f} s")
    for label, h in HEADERS.items():
        print(f"header {label}: {data.count(h)} hits")

    bursts = []  # [t_start, offset, length]
    last_t = None
    for t, off, n in index:
        if last_t is None or t - last_t > BURST_GAP_S:
            bursts.append([t, off, n])
        else:
            bursts[-1][2] += n
        last_t = t
    print(f"bursts (gap > {BURST_GAP_S * 1e3:.0f} ms): {len(bursts)}")
    print("first bursts:")
    for t, off, n in bursts[:12]:
        chunk = data[off:off + n]
        shown = chunk[:48].hex(" ")
        more = f" ... (+{n - 48} bytes)" if n > 48 else ""
        print(f"  t={t:9.4f}s  len={n:6d}  {shown}{more}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default=PORT)
    ap.add_argument("--baud", type=int, default=6_000_000)
    ap.add_argument("--seconds", type=float, default=120.0)
    ap.add_argument("--name", default="capture")
    args = ap.parse_args()

    session = os.path.join(ROOT, datetime.now().strftime("%Y-%m-%d"))
    os.makedirs(session, exist_ok=True)
    stem = os.path.join(session, f"{datetime.now():%H%M%S}_{args.name}_{args.baud}")

    # open at a standard rate, then switch to the requested one
    ser = serial.Serial(args.port, 4_000_000, bytesize=8, parity="N", stopbits=1, timeout=0.001)
    actual = set_baud(ser.fd, args.baud)
    if actual != args.baud:
        print(f"warning: asked for {args.baud} baud, driver reports {actual}", file=sys.stderr)
    try:
        ser.set_low_latency_mode(True)  # FTDI latency timer 16 ms -> 1 ms
    except Exception as e:  # noqa: BLE001
        print(f"warning: low-latency mode not set ({e}); timestamps are ~16 ms coarse", file=sys.stderr)
    time.sleep(0.1)  # let bytes received before the baud switch drain
    ser.reset_input_buffer()

    print(f"listening on {args.port} @ {args.baud} baud for {args.seconds:.0f} s -> {stem}.bin", flush=True)
    index = []
    buf = bytearray()
    t0 = time.monotonic()
    announced = False
    try:
        with open(stem + ".bin", "wb") as fbin, open(stem + ".idx", "w") as fidx:
            while (now := time.monotonic() - t0) < args.seconds:
                chunk = ser.read(65536)
                if not chunk:
                    continue
                t = time.monotonic() - t0
                if not announced:
                    print(f"first activity at t = {t:.3f} s", flush=True)
                    announced = True
                fidx.write(f"{t:.6f} {len(buf)} {len(chunk)}\n")
                fbin.write(chunk)
                index.append((t, len(buf), len(chunk)))
                buf += chunk
    except KeyboardInterrupt:
        print("stopped early")
    finally:
        ser.close()

    summarize(bytes(buf), index, args.baud)
    return 0


if __name__ == "__main__":
    sys.exit(main())

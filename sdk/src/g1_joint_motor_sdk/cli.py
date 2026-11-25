"""g1motor command line: scan, read, sniff."""
from __future__ import annotations

import argparse
import sys
import time

from . import units
from .bus import BusBusyError, MotorBus, MotorFaultError
from .serial_port import open_port


def _line(s) -> str:
    return (f"id {s.id}: mode {s.mode}  pos {s.pos:8d} ({units.pos_to_rad(s.pos):8.3f} rad)  vel {s.vel:6d}  "
            f"tau {s.tau:5d}  {s.voltage:4.1f} V  temps {s.temp_a}/{s.temp_b}"
            + (f"  FAULT 0x{s.error:x}" if s.fault else ""))


def cmd_scan(args) -> int:
    with MotorBus(args.port) as bus:
        found = bus.scan()
    if not found:
        print("no motors answered on ids 0-14")
        return 1
    for state in found.values():
        print(_line(state))
    return 0


def cmd_read(args) -> int:
    ids = [int(x) for x in args.ids.split(",")]
    with MotorBus(args.port) as bus:
        if args.zero_torque:
            bus.enable(ids)
        end, next_print = time.monotonic() + args.seconds, 0.0
        try:
            while time.monotonic() < end:
                states = [bus.read(i) for i in ids]
                if time.monotonic() >= next_print:
                    next_print = time.monotonic() + 0.2
                    print("   ".join(_line(s) if s else "no reply" for s in states), flush=True)
        except KeyboardInterrupt:
            pass
    return 0


def cmd_sniff(args) -> int:
    ser = open_port(args.port)
    end, total = time.monotonic() + args.seconds, 0
    with open(args.out, "wb") as f:
        try:
            while time.monotonic() < end:
                chunk = ser.read(65536)
                f.write(chunk)
                total += len(chunk)
        except KeyboardInterrupt:
            pass
    ser.close()
    print(f"{total} bytes written to {args.out}")
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="g1motor", description="Unofficial tools for Unitree G1 joint motors (RS-485).")
    ap.add_argument("--port", help="serial device; default: the only FTDI adapter plugged in")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("scan", help="find motors on a bus without a controller").set_defaults(fn=cmd_scan)
    p = sub.add_parser("read", help="print motor state continuously")
    p.add_argument("--ids", required=True, help="comma separated, e.g. 4,5,6")
    p.add_argument("--seconds", type=float, default=10.0)
    p.add_argument("--zero-torque", action="store_true", help="enable the motors with zero torque while reading")
    p.set_defaults(fn=cmd_read)
    p = sub.add_parser("sniff", help="record raw bus bytes without transmitting")
    p.add_argument("--seconds", type=float, default=10.0)
    p.add_argument("--out", required=True)
    p.set_defaults(fn=cmd_sniff)
    args = ap.parse_args(argv)
    try:
        return args.fn(args)
    except (BusBusyError, MotorFaultError) as e:
        print(f"error: {e}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())

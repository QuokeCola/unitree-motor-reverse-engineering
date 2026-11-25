"""g1motor command line: scan, read, sniff."""
from __future__ import annotations

import argparse
import math
import sys
import time

from . import units
from .bus import BusBusyError, MotorBus, MotorFaultError
from .joints import WRIST_GEAR_RATIOS, Joint
from .serial_port import open_port


def _line(s) -> str:
    return (f"id {s.id}: mode {s.mode}  pos {s.pos:8d} ({units.pos_to_rad(s.pos):8.3f} rad)  vel {s.vel:6d}  "
            f"tau {s.tau:5d}  {s.voltage:4.1f} V  coil {s.temp_coil} C  shell {s.temp_shell} C"
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


def cmd_limits(args) -> int:
    """Free the joints and record the lowest and highest position each one is moved to."""
    import json
    ids = [int(x) for x in args.ids.split(",")]
    low, high = {}, {}
    with MotorBus(args.port) as bus:
        bus.enable(ids)
        print(f"joints are free for {args.seconds:.0f} s: move each one to both end stops", flush=True)
        end = time.monotonic() + args.seconds
        try:
            while time.monotonic() < end:
                for i in ids:
                    s = bus.command(i)
                    if s:
                        low[i], high[i] = min(low.get(i, s.pos), s.pos), max(high.get(i, s.pos), s.pos)
        except KeyboardInterrupt:
            pass
    for i in ids:
        if i not in low:
            print(f"id {i}: no data")
            continue
        ratio = WRIST_GEAR_RATIOS.get(i)
        extra = f" = {math.degrees(Joint(ratio, low[i], high[i]).travel):.0f} deg at the output ({ratio}:1)" if ratio else ""
        print(f"id {i}: {low[i]} .. {high[i]} counts{extra}")
    if args.out:
        json.dump({str(i): [low[i], high[i]] for i in ids if i in low}, open(args.out, "w"))
        print("written to", args.out)
    return 0


def cmd_restart(args) -> int:
    ids = [int(x) for x in args.ids.split(",")]
    with MotorBus(args.port) as bus:
        before = {i: bus.read(i) for i in ids}
        after = bus.restart(ids)
    for i in ids:
        b, a = before.get(i), after.get(i)
        if not a:
            print(f"id {i}: no reply after restart")
            continue
        shift = f", position reading shifted by {a.pos - b.pos} counts" if b and abs(a.pos - b.pos) > 100_000 else ""
        print(f"id {i}: status 0x{b.error:x} -> 0x{a.error:x}{shift}" if b else _line(a))
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
    p = sub.add_parser("limits", help="free the joints and record the end stops you move them to")
    p.add_argument("--ids", required=True)
    p.add_argument("--seconds", type=float, default=60.0)
    p.add_argument("--out", help="JSON file to write, readable by load_joints()")
    p.set_defaults(fn=cmd_limits)
    p = sub.add_parser("restart", help="restart motors (clears a latched fault)")
    p.add_argument("--ids", required=True)
    p.set_defaults(fn=cmd_restart)
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

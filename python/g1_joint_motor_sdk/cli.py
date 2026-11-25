"""!
@file cli.py
@brief The `g1motor` command: scan, read, limits, restart, sniff.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time

from g1_joint_motor_sdk import joint as joint_lib
from g1_joint_motor_sdk import motor_bus
from g1_joint_motor_sdk import protocol
from g1_joint_motor_sdk import serial_port
from g1_joint_motor_sdk import units


def _parse_ids(text: str) -> list[int]:
    """!
    @brief Parses a comma-separated id list.
    @param text For example "4,5,6".
    @return The ids.
    """
    return [int(part) for part in text.split(",")]


def _format_state(state: protocol.MotorState) -> str:
    """!
    @brief Formats one motor state as a line of text.
    @param state State to show.
    @return The line.
    """
    line = (f"id {state.id}: mode {state.mode}  pos {state.pos:8d} "
            f"({units.pos_to_rad(state.pos):8.3f} rad)  vel {state.vel:6d}  "
            f"tau {state.tau:5d}  {state.voltage:4.1f} V  "
            f"coil {state.temp_coil} C  shell {state.temp_shell} C")
    if state.fault:
        line += f"  FAULT 0x{state.error:x}"
    return line


def run_scan(args: argparse.Namespace) -> int:
    """!
    @brief `g1motor scan`: lists the motors on the bus.
    @param args Parsed command line.
    @return Exit code; 1 if no motor answered.
    """
    with motor_bus.MotorBus(args.port) as bus:
        found = bus.scan()
    if not found:
        print("no motors answered on ids 0-14")
        return 1
    for state in found.values():
        print(_format_state(state))
    return 0


def run_read(args: argparse.Namespace) -> int:
    """!
    @brief `g1motor read`: prints motor state five times a second.
    @param args Parsed command line.
    @return Exit code.
    """
    ids = _parse_ids(args.ids)
    with motor_bus.MotorBus(args.port) as bus:
        if args.zero_torque:
            bus.enable(ids)
        end = time.monotonic() + args.seconds
        next_print = 0.0
        try:
            while time.monotonic() < end:
                states = [bus.read(motor_id) for motor_id in ids]
                if time.monotonic() >= next_print:
                    next_print = time.monotonic() + 0.2
                    print("   ".join(_format_state(state) if state
                                     else "no reply" for state in states),
                          flush=True)
        except KeyboardInterrupt:
            pass
    return 0


def run_limits(args: argparse.Namespace) -> int:
    """!
    @brief `g1motor limits`: frees the joints and records their end stops.
    @param args Parsed command line.
    @return Exit code.
    """
    ids = _parse_ids(args.ids)
    low: dict[int, int] = {}
    high: dict[int, int] = {}
    with motor_bus.MotorBus(args.port) as bus:
        bus.enable(ids)
        print(f"joints are free for {args.seconds:.0f} s: move each one to "
              "both end stops", flush=True)
        end = time.monotonic() + args.seconds
        try:
            while time.monotonic() < end:
                for motor_id in ids:
                    state = bus.command(motor_id)
                    if not state:
                        continue
                    low[motor_id] = min(low.get(motor_id, state.pos),
                                        state.pos)
                    high[motor_id] = max(high.get(motor_id, state.pos),
                                         state.pos)
        except KeyboardInterrupt:
            pass
    for motor_id in ids:
        if motor_id not in low:
            print(f"id {motor_id}: no data")
            continue
        line = f"id {motor_id}: {low[motor_id]} .. {high[motor_id]} counts"
        ratio = joint_lib.WRIST_GEAR_RATIOS.get(motor_id)
        if ratio and high[motor_id] > low[motor_id]:
            travel = joint_lib.Joint(ratio, low[motor_id],
                                     high[motor_id]).travel
            line += (f" = {math.degrees(travel):.0f} deg at the output "
                     f"({ratio}:1)")
        print(line)
    if args.out:
        with open(args.out, "w") as out_file:
            json.dump({str(motor_id): [low[motor_id], high[motor_id]]
                       for motor_id in ids if motor_id in low}, out_file)
        print("written to", args.out)
    return 0


def run_restart(args: argparse.Namespace) -> int:
    """!
    @brief `g1motor restart`: restarts motors, clearing a latched fault.
    @param args Parsed command line.
    @return Exit code.
    """
    ids = _parse_ids(args.ids)
    with motor_bus.MotorBus(args.port) as bus:
        before = {motor_id: bus.read(motor_id) for motor_id in ids}
        after = bus.restart(ids)
    for motor_id in ids:
        old, new = before.get(motor_id), after.get(motor_id)
        if not new:
            print(f"id {motor_id}: no reply after restart")
        elif not old:
            print(_format_state(new))
        else:
            shift = new.pos - old.pos
            note = ""
            if abs(shift) > 100_000:
                note = f", position reading shifted by {shift} counts"
            print(f"id {motor_id}: status 0x{old.error:x} -> "
                  f"0x{new.error:x}{note}")
    return 0


def run_sniff(args: argparse.Namespace) -> int:
    """!
    @brief `g1motor sniff`: records raw bus bytes without transmitting.
    @param args Parsed command line.
    @return Exit code.
    """
    port = serial_port.open_port(args.port)
    end = time.monotonic() + args.seconds
    total = 0
    with open(args.out, "wb") as out_file:
        try:
            while time.monotonic() < end:
                chunk = port.read(65536)
                out_file.write(chunk)
                total += len(chunk)
        except KeyboardInterrupt:
            pass
    port.close()
    print(f"{total} bytes written to {args.out}")
    return 0


def main(argv: list[str] | None = None) -> int:
    """!
    @brief Entry point of the `g1motor` command.
    @param argv Arguments; None uses the process arguments.
    @return Exit code.
    """
    parser = argparse.ArgumentParser(
        prog="g1motor",
        description="Unofficial tools for Unitree G1 joint motors (RS-485).")
    parser.add_argument(
        "--port", help="serial device; default: the only FTDI adapter")
    commands = parser.add_subparsers(dest="command", required=True)

    scan = commands.add_parser("scan", help="find motors on the bus")
    scan.set_defaults(run=run_scan)

    read = commands.add_parser("read", help="print motor state continuously")
    read.add_argument("--ids", required=True, help="for example 4,5,6")
    read.add_argument("--seconds", type=float, default=10.0)
    read.add_argument("--zero-torque", action="store_true",
                      help="enable the motors with zero torque while reading")
    read.set_defaults(run=run_read)

    limits = commands.add_parser(
        "limits", help="free the joints and record their end stops")
    limits.add_argument("--ids", required=True)
    limits.add_argument("--seconds", type=float, default=60.0)
    limits.add_argument("--out", help="JSON file readable by load_joints()")
    limits.set_defaults(run=run_limits)

    restart = commands.add_parser(
        "restart", help="restart motors (clears a latched fault)")
    restart.add_argument("--ids", required=True)
    restart.set_defaults(run=run_restart)

    sniff = commands.add_parser(
        "sniff", help="record raw bus bytes without transmitting")
    sniff.add_argument("--seconds", type=float, default=10.0)
    sniff.add_argument("--out", required=True)
    sniff.set_defaults(run=run_sniff)

    args = parser.parse_args(argv)
    try:
        return args.run(args)
    except (motor_bus.BusBusyError, motor_bus.MotorFaultError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""Probe untried mode values (bits 4-6 of the id/mode byte) on ONE motor, one at a time.

Each mode is sent with every payload field zero for a short burst while the reply is watched.
The burst stops at once, with idle frames, if the motor starts to move.

  mode_probe.py --id 4 --modes 7,6,5,4,3,2
"""
import argparse
import sys
import time

import serial

from g1bus import REPLY_LEN, build_command, parse_reply
from sniff import PORT, set_baud

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument("--id", type=int, default=4)
ap.add_argument("--modes", default="7,6,5,4,3,2")
ap.add_argument("--frames", type=int, default=40, help="frames per burst (about 1 ms each)")
ap.add_argument("--watch", type=float, default=1.5, help="seconds of idle polling after each burst")
ap.add_argument("--no-flag", action="store_true", help="send with bit 7 clear")
args = ap.parse_args()

ser = serial.Serial(PORT, 4_000_000, timeout=0.0005)
set_baud(ser.fd, 6_000_000)
ser.set_low_latency_mode(True)
time.sleep(0.1)
ser.reset_input_buffer()


def xfer(cmd: bytes):
    """Returns (parsed reply or None, raw bytes received)."""
    ser.reset_input_buffer()
    ser.write(cmd)
    end, rx = time.monotonic() + 0.004, b""
    while time.monotonic() < end:
        rx += ser.read(256)
        at = rx.find(b"\xfc\xee")
        if at >= 0 and len(rx) - at >= REPLY_LEN:
            r = parse_reply(rx[at:at + REPLY_LEN])
            if r:
                return r, rx
    return None, rx


def idle(n=20):
    last = None
    for _ in range(n):
        r, _ = xfer(build_command(args.id))
        last = r or last
    return last


def show(r):
    return f"mode {r['mode']} status 0x{r['word14']:x} pos {r['pos']} vel {r['vel']} tau {r['tau']}" if r else "no reply"


base = idle()
print(f"motor {args.id} before: {show(base)}")
if not base:
    sys.exit("motor does not answer")

for mode in [int(x) for x in args.modes.split(",")]:
    start = idle(5)
    seen_modes, statuses, odd, moved, sent = set(), set(), [], False, 0
    peak_vel, last = 0, start
    for n in range(args.frames):
        byte3 = 0 if args.no_flag else (0x80 if (n // 8) % 2 == 0 else 0x90)
        r, raw = xfer(build_command(args.id, mode=mode, flag=not args.no_flag, byte3=byte3))
        sent += 1
        if r is None:
            if raw and len(odd) < 3:
                odd.append(raw.hex(" "))
            continue
        last = r
        seen_modes.add(r["mode"]); statuses.add(r["word14"]); peak_vel = max(peak_vel, abs(r["vel"]))
        if abs(r["vel"]) > 300:   # real motion (a position jump alone can be a restart re-referencing)
            moved = True
            break
    # watch what it does once we go back to idle polling
    t0, after_modes, max_dev, max_vel = time.monotonic(), set(), 0, 0
    while time.monotonic() - t0 < args.watch:
        r, _ = xfer(build_command(args.id))
        if r:
            after_modes.add(r["mode"]); last = r
            max_dev = max(max_dev, abs(r["pos"] - start["pos"])); max_vel = max(max_vel, abs(r["vel"]))
    print(f"mode {mode}: sent {sent} frames | reply modes during {sorted(seen_modes)} status {[hex(s) for s in sorted(statuses)]} "
          f"peak vel {peak_vel}{' MOVED - stopped early' if moved else ''} | afterwards: modes {sorted(after_modes)}, "
          f"moved {max_dev} counts, peak vel {max_vel} | now {show(last)}", flush=True)
    for o in odd:
        print("    unparsed reply bytes:", o)
    if moved or max_vel > 300:
        print("    motion seen; stopping the probe here")
        break

print(f"motor {args.id} after:  {show(idle())}")
ser.close()

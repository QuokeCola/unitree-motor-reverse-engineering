#!/usr/bin/env python3
"""Compare how one motor responds to the same small commands in different modes.

For each mode the same short tests are run from rest near mid-travel: torque only, torque with
damping, a small position step, and a velocity command. Mode 1 is the reference. Every test is
cut short, and the joint braked gently, if it moves faster or further than the limits below.
"""
import argparse
import json
import os
import sys
import time

import serial

from g1bus import REPLY_LEN, build_command, parse_reply
from sniff import PORT, set_baud

HERE = os.path.dirname(os.path.abspath(__file__))
WRAP = {4: 16 * 32768, 5: 25 * 32768, 6: 25 * 32768}
MAX_VEL = 1200        # velocity counts (about 150,000 position counts/s): stop the test above this
MAX_TRAVEL = 40_000   # position counts from the test's start

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument("--id", type=int, default=4)
ap.add_argument("--modes", default="1,3,4,5")
ap.add_argument("--seconds", type=float, default=0.4, help="length of each test")
args = ap.parse_args()
mid_ = args.id

ser = serial.Serial(PORT, 4_000_000, timeout=0.0005)
set_baud(ser.fd, 6_000_000)
ser.set_low_latency_mode(True)
time.sleep(0.1)
ser.reset_input_buffer()
count = 0


def xfer(mode=0, flag=None, **fields):
    global count
    count += 1
    flag = (mode != 0) if flag is None else flag
    byte3 = (0x80 if (count // 8) % 2 == 0 else 0x90) if flag else 0
    ser.reset_input_buffer()
    ser.write(build_command(mid_, mode=mode, flag=flag, byte3=byte3, **fields))
    end, rx = time.monotonic() + 0.004, b""
    while time.monotonic() < end:
        rx += ser.read(256)
        at = rx.find(b"\xfc\xee")
        if at >= 0 and len(rx) - at >= REPLY_LEN:
            r = parse_reply(rx[at:at + REPLY_LEN])
            if r:
                return r
    return None


def poll(n=20, **kw):
    last = None
    for _ in range(n):
        last = xfer(**kw) or last
    return last


def settle():
    """Gentle damping in run mode until the joint is at rest, then idle."""
    for _ in range(200):
        r = xfer(mode=1, kd=10)
        if r and abs(r["vel"]) < 15:
            break
    return poll(30)


def go_to(target, speed=50_000):
    r = poll(10, mode=1)
    x, t_last = float(r["pos"]), time.monotonic()
    while abs(target - x) > 1:
        now = time.monotonic()
        step = min(speed * (now - t_last), abs(target - x))
        x, t_last = x + (step if target > x else -step), now
        xfer(mode=1, pos=round(x), kp=300, kd=40)
    for _ in range(150):
        xfer(mode=1, pos=round(target), kp=300, kd=40)
    return settle()


state = poll()
if not state:
    sys.exit("motor does not answer")
if state["word14"]:
    sys.exit(f"motor reports status 0x{state['word14']:x}; restart it first")
lo, hi = json.load(open(os.path.join(HERE, "calibration", "limits.json")))[str(mid_)]
k = round((state["pos"] - (lo + hi) / 2) / WRAP[mid_])
lo, hi = lo + k * WRAP[mid_], hi + k * WRAP[mid_]
if not lo <= state["pos"] <= hi:
    sys.exit("position does not fit the stored end stops")
centre = (lo + hi) / 2
print(f"motor {mid_}: end stops {lo} .. {hi}, centre {centre:.0f}, now at {state['pos']}")

# name, fields as a function of the start position
TESTS = [
    ("torque +15",                 lambda p: dict(tau=15)),
    ("torque -15",                 lambda p: dict(tau=-15)),
    ("torque +15, kd 20",          lambda p: dict(tau=15, kd=20)),
    ("pos +4000, kp 100, kd 10",   lambda p: dict(pos=p + 4000, kp=100, kd=10)),
    ("pos +4000, kp 100, kd 0",    lambda p: dict(pos=p + 4000, kp=100)),
    ("vel +300, kd 20",            lambda p: dict(vel=300, kd=20)),
    ("vel -300, kd 20",            lambda p: dict(vel=-300, kd=20)),
    ("kd 60 only",                 lambda p: dict(kd=60)),
]

print(f"{'mode':>4}  {'test':26s} {'reply mode':>10} {'moved':>8} {'end vel':>8} {'peak vel':>9} {'mean tau':>9} {'status':>7}  note")
for mode in [int(x) for x in args.modes.split(",")]:
    for name, make in TESTS:
        start = go_to(centre)
        if not start:
            sys.exit("lost the motor")
        fields = make(start["pos"])
        t0, modes, vels, taus, last, note = time.monotonic(), set(), [], [], start, ""
        while time.monotonic() - t0 < args.seconds:
            r = xfer(mode=mode, **fields)
            if not r:
                continue
            last = r
            modes.add(r["mode"]); vels.append(r["vel"]); taus.append(r["tau"])
            if abs(r["vel"]) > MAX_VEL or abs(r["pos"] - start["pos"]) > MAX_TRAVEL:
                note = f"stopped early at {time.monotonic() - t0:.2f} s"
                break
        moved = last["pos"] - start["pos"]
        tail = vels[-20:] or [0]
        print(f"{mode:>4}  {name:26s} {str(sorted(modes)):>10} {moved:>8d} {sum(tail) / len(tail):>8.0f} "
              f"{max(vels, key=abs) if vels else 0:>9d} {sum(taus) / max(len(taus), 1):>9.1f} {hex(last['word14']):>7}  {note}",
              flush=True)
        after = settle()
        if after and after["word14"]:
            print("      status word went non-zero; stopping")
            ser.close()
            sys.exit(1)
    print()
go_to(centre)
poll(30)
ser.close()

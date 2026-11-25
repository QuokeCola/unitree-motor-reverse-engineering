#!/usr/bin/env python3
"""With motors in a latched fault, try bus commands that might clear it. Stops at the first that works.

Only frames the G1 controller itself was seen sending are used (no untried mode values).
"""
import sys
import time

import serial

from g1bus import build_command
from sniff import PORT, set_baud
from zero_torque import exchange

IDS = [4, 5, 6]
ser = serial.Serial(PORT, 4_000_000, timeout=0.0005)
set_baud(ser.fd, 6_000_000)
ser.set_low_latency_mode(True)
time.sleep(0.1)
ser.reset_input_buffer()


def toggle(n):
    return 0x80 if (n // 8) % 2 == 0 else 0x90


def status():
    out = {}
    for i in IDS:
        r = None
        for _ in range(3):
            r = exchange(ser, build_command(i)) or r
        out[i] = (r["word14"], r["mode"]) if r else None
    return out


def can_enable():
    """Short zero-torque enable attempt; returns the modes reported, then goes back to idle."""
    modes = {}
    for n in range(150):
        for i in IDS:
            r = exchange(ser, build_command(i, mode=1, flag=True, byte3=toggle(n)))
            if r:
                modes[i] = r["mode"]
    for _ in range(20):
        for i in IDS:
            exchange(ser, build_command(i))
    return modes


def send_for(seconds, make):
    end, n = time.monotonic() + seconds, 0
    while time.monotonic() < end:
        for i in IDS:
            exchange(ser, make(i, n))
        n += 1
    return n


attempts = [
    ("run-mode damping (0x9N, kd 25), 1 s", lambda: send_for(1.0, lambda i, n: build_command(i, mode=1, flag=True, byte3=toggle(n), kd=25))),
    ("idle + flag with kd 25 (0x8N), 1 s", lambda: send_for(1.0, lambda i, n: build_command(i, mode=0, flag=True, byte3=toggle(n), kd=25))),
    ("boot probe (0x0N, byte3 0, torque -128), 4 rounds", lambda: [exchange(ser, build_command(i, tau=-128)) for _ in range(4) for i in IDS]),
    ("boot probe for 1 s", lambda: send_for(1.0, lambda i, n: build_command(i, tau=-128))),
    ("boot probe 4 rounds, then 7 s silence (as at robot boot)", lambda: ([exchange(ser, build_command(i, tau=-128)) for _ in range(4) for i in IDS], time.sleep(7.0))),
    ("15 s of silence", lambda: time.sleep(15.0)),
]

print("start:", {i: (f"status 0x{s[0]:x}, mode {s[1]}" if s else "no reply") for i, s in status().items()})
for name, action in attempts:
    action()
    st = status()
    cleared = all(s and s[0] == 0 for s in st.values())
    modes = can_enable()
    print(f"{name:58s} -> status {[hex(s[0]) if s else None for s in st.values()]}  enable gives modes {list(modes.values())}", flush=True)
    if cleared or all(m == 1 for m in modes.values()):
        print("CLEARED by:", name)
        break
else:
    print("none of these cleared the fault")
ser.close()

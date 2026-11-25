#!/usr/bin/env python3
"""Find motors on a bus that has no controller.

Listens first; if the bus is silent, polls ids 0-14 with the idle frame the G1 controller
itself sends during boot (mode 0, every field zero) and reports who answers.
"""
import argparse
import sys
import time

import serial

from g1bus import REPLY_LEN, build_command, parse_reply
from sniff import PORT, set_baud


def read_for(ser, seconds: float) -> bytes:
    end, buf = time.monotonic() + seconds, b""
    while time.monotonic() < end:
        buf += ser.read(4096)
    return buf


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", default=PORT)
    ap.add_argument("--baud", type=int, default=6_000_000)
    ap.add_argument("--tries", type=int, default=3)
    args = ap.parse_args()

    ser = serial.Serial(args.port, 4_000_000, timeout=0.001)
    set_baud(ser.fd, args.baud)
    ser.set_low_latency_mode(True)
    time.sleep(0.1)
    ser.reset_input_buffer()

    heard = read_for(ser, 1.0)
    if heard:
        print(f"bus is not silent ({len(heard)} bytes in 1 s): something is already talking; not transmitting")
        print(heard[:64].hex(" "))
        return 1
    print("bus silent for 1 s; polling ids 0-14 in mode 0 with zero payload")

    found = {}
    for motor_id in range(15):   # 15 is the broadcast id in the public GO-M8010-6 protocol; skipped
        cmd = build_command(motor_id)
        for _ in range(args.tries):
            ser.reset_input_buffer()
            ser.write(cmd)
            ser.flush()
            rx = read_for(ser, 0.01)
            at = rx.find(b"\xfc\xee")
            reply = parse_reply(rx[at:at + REPLY_LEN]) if at >= 0 else None
            if reply:
                found[motor_id] = reply
                break
            if rx and rx != cmd:
                print(f"  id {motor_id}: unparsed bytes {rx.hex(' ')}")
    ser.close()

    if not found:
        print("no replies")
    for motor_id, r in found.items():
        print(f"id {motor_id}: mode {r['mode']}  temps {r['temp_a']}/{r['temp_b']}  byte5 {r['b5']}  "
              f"pos {r['pos']}  vel {r['vel']}  tau {r['tau']}  word18 {r['u18']}\n      {r['raw'].hex(' ')}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

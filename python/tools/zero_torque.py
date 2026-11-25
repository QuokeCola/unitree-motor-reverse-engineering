#!/usr/bin/env python3
"""Put bench motors into zero-torque mode the way the G1 controller does.

Polls each id continuously with mode 1 and every command field zero (no torque, no
gains), prints telemetry, and returns the motors to mode 0 on exit.
"""
import argparse
import sys
import time

import serial

from g1bus import REPLY_LEN, build_command, parse_reply
from sniff import PORT, set_baud


def exchange(ser, cmd: bytes, wait: float = 0.004):
    ser.write(cmd)
    end, rx = time.monotonic() + wait, b""
    while time.monotonic() < end:
        rx += ser.read(256)
        at = rx.find(b"\xfc\xee")
        if at >= 0 and len(rx) - at >= REPLY_LEN:
            return parse_reply(rx[at:at + REPLY_LEN])
    return None


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--ids", default="4,5,6")
    ap.add_argument("--seconds", type=float, default=5.0)
    ap.add_argument("--port", default=PORT)
    ap.add_argument("--save-limits", help="write the lowest and highest position seen per id to this JSON file")
    ap.add_argument("--log", help="CSV file for every reply: t,id,mode,pos,vel,tau,temp_a,temp_b")
    args = ap.parse_args()
    ids = [int(x) for x in args.ids.split(",")]

    ser = serial.Serial(args.port, 4_000_000, timeout=0.0005)
    set_baud(ser.fd, 6_000_000)
    ser.set_low_latency_mode(True)
    time.sleep(0.1)
    ser.reset_input_buffer()

    stats = {i: dict(sent=0, got=0, last=None, modes=set(), lo=None, hi=None) for i in ids}
    log = open(args.log, "w") if args.log else None
    if log:
        log.write("t,id,mode,pos,vel,tau,temp_a,temp_b\n")
    t_log0 = time.monotonic()

    def cycle(mode: int, flag: bool, n: int) -> None:
        byte3 = 0 if not flag else (0x80 if (n // 8) % 2 == 0 else 0x90)   # robot toggles 80/90 every ~8 frames
        for i in ids:
            r = exchange(ser, build_command(i, mode=mode, flag=flag, byte3=byte3))
            s = stats[i]
            s["sent"] += 1
            if r and r["id"] == i:
                s["got"] += 1
                s["last"] = r
                s["modes"].add(r["mode"])
                s["lo"] = r["pos"] if s["lo"] is None else min(s["lo"], r["pos"])
                s["hi"] = r["pos"] if s["hi"] is None else max(s["hi"], r["pos"])
                if log:
                    log.write(f"{time.monotonic() - t_log0:.5f},{i},{r['mode']},{r['pos']},{r['vel']},{r['tau']},"
                              f"{r['temp_a']},{r['temp_b']}\n")

    n, t0, next_print = 0, time.monotonic(), 0.0
    try:
        # same order as the controller at boot: mode 0, then flag bit, then mode 1
        for _ in range(20):
            cycle(0, False, n); n += 1
        for _ in range(50):
            cycle(0, True, n); n += 1
        t0 = time.monotonic()
        while time.monotonic() - t0 < args.seconds:
            cycle(1, True, n); n += 1
            if time.monotonic() - t0 >= next_print:
                next_print += 5.0
                print(f"t={time.monotonic() - t0:5.1f}s  " + "   ".join(
                    f"id{i}: mode {s['last']['mode']} pos {s['last']['pos']} range {s['lo']}..{s['hi']}"
                    if s["last"] else f"id{i}: no reply" for i, s in stats.items()), flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(20):   # back to idle
            cycle(0, False, 0)
        rate = n / max(time.monotonic() - t0, 1e-6)
        for i, s in stats.items():
            print(f"id {i}: {s['got']}/{s['sent']} replies, modes seen {sorted(s['modes'])}, "
                  f"final mode {s['last']['mode'] if s['last'] else '?'}")
            if s["lo"] is not None:
                span = s["hi"] - s["lo"]
                print(f"      position {s['lo']} .. {s['hi']} counts, span {span} = {span / 32768:.3f} rotor turns "
                      f"(assuming 32768 counts per turn)")
        print(f"about {rate:.0f} polls per second per motor")
        if args.save_limits:
            import json
            json.dump({str(i): [s["lo"], s["hi"]] for i, s in stats.items() if s["lo"] is not None},
                      open(args.save_limits, "w"))
            print("limits written to", args.save_limits)
        if log:
            log.close()
        ser.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())

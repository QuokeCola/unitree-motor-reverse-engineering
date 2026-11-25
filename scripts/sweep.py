#!/usr/bin/env python3
"""Drive bench motors slowly across their range of motion, one joint at a time.

Each joint goes from where it is to the low end, to the high end, and back, on a
constant-speed position ramp with soft gains. The other joints are held where they are.
Stops and returns every motor to idle (mode 0) on any abort condition or Ctrl-C.

  sweep.py --dry-run           show the plan, transmit nothing
  sweep.py --nudge 5000        small test: move each joint +/- 5000 counts only
  sweep.py                     full sweep
"""
import argparse
import json
import math
import sys
import time

import serial

from g1bus import build_command
from sniff import PORT, set_baud
from zero_torque import exchange

# End stops found by hand in zero-torque mode, capture 134738_bench_range_ids456.csv (raw counts).
LIMITS = {4: (-95045, 235962), 5: (322, 426057), 6: (-190234, 236866)}
VEL_PER_POS = 128.0   # position counts per second for one velocity count (checked on robot data)


class Abort(Exception):
    pass


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ids", default="4,5,6")
    ap.add_argument("--margin", type=float, default=0.10, help="fraction of the span kept clear of each end stop")
    ap.add_argument("--speed", type=float, default=50_000, help="ramp speed, position counts per second")
    ap.add_argument("--kp", default="20", help="one value, or one per id separated by commas")
    ap.add_argument("--kd", default="1", help="one value, or one per id separated by commas")
    ap.add_argument("--max-lag", type=int, default=30_000, help="abort if |target - measured| exceeds this")
    ap.add_argument("--max-tau", type=int, default=300, help="abort if |reported torque| exceeds this (raw counts)")
    ap.add_argument("--nudge", type=int, help="instead of the full range, move +/- this many counts")
    ap.add_argument("--together", action="store_true", help="move all joints at once instead of one at a time")
    ap.add_argument("--sine", type=float, metavar="SECONDS",
                    help="all joints follow a sine between their low and high targets for this long")
    ap.add_argument("--period", type=float, default=8.0, help="sine period in seconds")
    ap.add_argument("--ff-coulomb", default="0", help="friction feed-forward: torque counts at any speed, per id")
    ap.add_argument("--ff-viscous", default="0", help="friction feed-forward: torque counts per 100,000 counts/s, per id")
    ap.add_argument("--ff-table", help="JSON from build_ff_table.py: torque needed along the travel, per direction")
    ap.add_argument("--ff-gain", type=float, default=1.0, help="fraction of the friction feed-forward to apply")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--log")
    ap.add_argument("--port", default=PORT)
    args = ap.parse_args()
    ids = [int(x) for x in args.ids.split(",")]

    def per_id(text: str) -> dict:
        vals = [int(x) for x in text.split(",")]
        if len(vals) == 1:
            vals *= len(ids)
        if len(vals) != len(ids):
            ap.error("give one gain, or one per id")
        return dict(zip(ids, vals))

    kp, kd = per_id(args.kp), per_id(args.kd)
    ff_c = {i: float(x) for i, x in zip(ids, (args.ff_coulomb.split(",") * len(ids))[:len(ids)])}
    ff_v = {i: float(x) for i, x in zip(ids, (args.ff_viscous.split(",") * len(ids))[:len(ids)])}

    table = json.load(open(args.ff_table)) if args.ff_table else {}

    def lookup(tab: dict, key: str, pos: float) -> float:
        x = (pos - tab["start"]) / tab["step"]
        k = min(max(int(x), 0), len(tab[key]) - 2)
        f = min(max(x - k, 0.0), 1.0)
        return tab[key][k] * (1 - f) + tab[key][k + 1] * f

    def friction_ff(i: int, vel_counts: float, pos: float) -> int:
        """Torque command that cancels the modelled friction at the commanded speed and position."""
        v = vel_counts * VEL_PER_POS                       # position counts per second
        direction = math.tanh(v / 5000.0)
        tab = table.get(str(i))
        if tab:   # table already contains the direction-dependent friction measured on a slow pass
            up, down = lookup(tab, "up", pos), lookup(tab, "down", pos)
            base = 0.5 * (up + down) + 0.5 * (up - down) * direction
        else:
            base = ff_c[i] * direction
        return round(args.ff_gain * (base + ff_v[i] * v / 1e5))

    targets = {}
    for i in ids:
        lo, hi = LIMITS[i]
        pad = args.margin * (hi - lo)
        targets[i] = (round(lo + pad), round(hi - pad))
    print(f"gains kp {kp} kd {kd}, speed {args.speed:.0f} counts/s "
          f"({args.speed / 32768:.2f} rotor turns/s if 32768 counts per turn)")
    for i in ids:
        lo, hi = targets[i]
        what = f"+/- {args.nudge} counts around the start" if args.nudge else f"{lo} .. {hi}"
        print(f"  id {i}: end stops {LIMITS[i][0]} .. {LIMITS[i][1]}, will move {what}"
              + ("" if args.nudge else f", about {2 * (hi - lo) / args.speed:.0f} s"))
    if args.sine:
        for i in ids:
            amp = (targets[i][1] - targets[i][0]) / 2
            print(f"  id {i}: sine amplitude {amp:.0f} counts, period {args.period} s, "
                  f"peak speed {2 * math.pi * amp / args.period:.0f} counts/s, for {args.sine:.0f} s")
    if args.dry_run:
        return 0

    ser = serial.Serial(args.port, 4_000_000, timeout=0.0005)
    set_baud(ser.fd, 6_000_000)
    ser.set_low_latency_mode(True)
    time.sleep(0.1)
    ser.reset_input_buffer()
    log = open(args.log, "w") if args.log else None
    if log:
        log.write("t,id,target,pos,vel,tau\n")

    state = {i: dict(pos=None, target=None, vel=0, missed=0, peak_tau=0, peak_lag=0) for i in ids}
    n = 0
    t_start = time.monotonic()

    def cycle(mode: int, flag: bool, active: bool) -> None:
        nonlocal n
        n += 1
        byte3 = 0 if not flag else (0x80 if (n // 8) % 2 == 0 else 0x90)
        for i in ids:
            s = state[i]
            if active:
                cmd = build_command(i, mode=mode, flag=flag, byte3=byte3, pos=round(s["target"]),
                                    vel=round(s["vel"]), kp=kp[i], kd=kd[i], tau=friction_ff(i, s["vel"], s["target"]))
            else:
                cmd = build_command(i, mode=mode, flag=flag, byte3=byte3)
            r = exchange(ser, cmd)
            if not r or r["id"] != i:
                s["missed"] += 1
                if s["missed"] > 10:
                    raise Abort(f"id {i}: no reply for 10 polls")
                continue
            s["missed"] = 0
            s["pos"] = r["pos"]
            if log and active:
                log.write(f"{time.monotonic() - t_start:.5f},{i},{round(s['target'])},{r['pos']},{r['vel']},{r['tau']}\n")
            if active:
                lag = abs(s["target"] - r["pos"])
                s["peak_lag"], s["peak_tau"] = max(s["peak_lag"], lag), max(s["peak_tau"], abs(r["tau"]))
                lo, hi = LIMITS[i]
                if lag > args.max_lag:
                    raise Abort(f"id {i}: lag {lag:.0f} counts (target {s['target']:.0f}, measured {r['pos']})")
                if abs(r["tau"]) > args.max_tau:
                    raise Abort(f"id {i}: torque reading {r['tau']}")
                if not lo <= r["pos"] <= hi:
                    raise Abort(f"id {i}: position {r['pos']} outside the measured end stops")

    def ramp(i: int, goal: float) -> None:
        s = state[i]
        direction = 1 if goal > s["target"] else -1
        s["vel"] = direction * args.speed / VEL_PER_POS
        last = time.monotonic()
        while (goal - s["target"]) * direction > 0:
            now = time.monotonic()
            s["target"] = s["target"] + direction * min(args.speed * (now - last), abs(goal - s["target"]))
            last = now
            cycle(1, True, True)
        s["vel"] = 0
        for _ in range(100):   # settle about 0.3 s
            cycle(1, True, True)

    def ramp_all(goals: dict) -> None:
        """Move every joint to its goal over the same time; the longest move runs at --speed."""
        starts = {i: state[i]["target"] for i in ids}
        duration = max(abs(goals[i] - starts[i]) for i in ids) / args.speed
        if duration <= 0:
            return
        for i in ids:
            state[i]["vel"] = (goals[i] - starts[i]) / duration / VEL_PER_POS
        t_begin = time.monotonic()
        while True:
            u = min((time.monotonic() - t_begin) / duration, 1.0)
            for i in ids:
                state[i]["target"] = starts[i] + u * (goals[i] - starts[i])
            cycle(1, True, True)
            if u >= 1.0:
                break
        for i in ids:
            state[i]["vel"] = 0
        for _ in range(100):
            cycle(1, True, True)

    def sine_all(seconds: float) -> None:
        """Sine about the middle of each range. Amplitude fades in and out over one period."""
        mid = {i: (targets[i][0] + targets[i][1]) / 2 for i in ids}
        amp = {i: (targets[i][1] - targets[i][0]) / 2 for i in ids}
        ramp_all(mid)
        t_begin = time.monotonic()
        w = 2 * math.pi / args.period
        while True:
            t = time.monotonic() - t_begin
            if t >= seconds:
                break
            # raised-cosine fade in/out; d_env is its exact time derivative
            if t < args.period:
                fade, sign = t / args.period, 1.0
            elif seconds - t < args.period:
                fade, sign = (seconds - t) / args.period, -1.0
            else:
                fade, sign = 1.0, 0.0
            envelope = 0.5 - 0.5 * math.cos(math.pi * fade)
            d_env = sign * 0.5 * math.pi / args.period * math.sin(math.pi * fade)
            for i in ids:
                state[i]["target"] = mid[i] + envelope * amp[i] * math.sin(w * t)
                # exact derivative of the target, so the feed-forward is free of timing jitter
                state[i]["vel"] = amp[i] * (d_env * math.sin(w * t) + envelope * w * math.cos(w * t)) / VEL_PER_POS
            cycle(1, True, True)
        for i in ids:
            state[i]["vel"] = 0

    try:
        for _ in range(20):
            cycle(0, False, False)
        for _ in range(50):
            cycle(0, True, False)
        for i in ids:
            if state[i]["pos"] is None:
                raise Abort(f"id {i}: no reply")
            state[i]["target"] = float(state[i]["pos"])   # hold where it is
        print("start positions: " + "  ".join(f"id{i} {state[i]['pos']}" for i in ids), flush=True)
        if args.sine:
            homes = {i: state[i]["target"] for i in ids}
            sine_all(args.sine)
            ramp_all(homes)
            for i in ids:
                print(f"id {i}: peak lag {state[i]['peak_lag']:.0f}, peak torque reading {state[i]['peak_tau']}, "
                      f"back at {state[i]['pos']} (started {homes[i]:.0f})")
        elif args.together:
            homes = {i: state[i]["target"] for i in ids}
            if args.nudge:
                lows = {i: max(homes[i] - args.nudge, LIMITS[i][0] + 2000) for i in ids}
                highs = {i: min(homes[i] + args.nudge, LIMITS[i][1] - 2000) for i in ids}
            else:
                lows = {i: targets[i][0] for i in ids}
                highs = {i: targets[i][1] for i in ids}
            for name, goals in (("low ends", lows), ("high ends", highs), ("start", homes)):
                ramp_all(goals)
                print(f"all joints at {name}: " + "  ".join(
                    f"id{i} target {goals[i]:.0f} measured {state[i]['pos']}" for i in ids), flush=True)
            for i in ids:
                print(f"id {i}: peak lag {state[i]['peak_lag']:.0f}, peak torque reading {state[i]['peak_tau']}")
        for i in ([] if args.together or args.sine else ids):
            home = state[i]["target"]
            if args.nudge:
                lo = max(home - args.nudge, LIMITS[i][0] + 2000)
                hi = min(home + args.nudge, LIMITS[i][1] - 2000)
            else:
                lo, hi = targets[i]
            for goal in (lo, hi, home):
                ramp(i, goal)
                s = state[i]
                print(f"id {i}: target {goal:.0f} reached, measured {s['pos']}, "
                      f"peak lag {s['peak_lag']:.0f}, peak torque reading {s['peak_tau']}", flush=True)
        print("sweep complete")
        code = 0
    except Abort as e:
        print(f"ABORTED: {e}")
        code = 2
    except KeyboardInterrupt:
        print("stopped by Ctrl-C")
        code = 130
    finally:
        try:
            if code != 0:   # brake first: pure damping for about 0.5 s, so nothing coasts into an end stop
                for _ in range(150):
                    n += 1
                    for i in ids:
                        exchange(ser, build_command(i, mode=1, flag=True, byte3=0x80 if (n // 8) % 2 == 0 else 0x90,
                                                    kd=max(kd[i], 80)))
                print("braked with damping before idle")
            for _ in range(30):
                cycle(0, False, False)
        except Abort:
            pass
        print("motors returned to idle (mode 0)")
        ser.close()
        if log:
            log.close()
    return code


if __name__ == "__main__":
    sys.exit(main())

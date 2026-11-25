"""Slow sine on the bench wrist joints, in output-shaft degrees, using stored end stops.

    g1motor limits --ids 4,5,6 --out limits.json      # once after every power-on: move joints to both stops
    python sine.py limits.json [PERIOD_S] [SECONDS]

Uses 80 % of each joint's travel. Gains are the ones tuned on that wrist at 24 V.
"""
import math
import sys
import time

from g1_joint_motor_sdk import MotorBus, load_joints, units

period = float(sys.argv[2]) if len(sys.argv) > 2 else 8.0
seconds = float(sys.argv[3]) if len(sys.argv) > 3 else 3 * period
KP, KD, MAX_LAG = 300, {4: 40, 5: 80, 6: 80}, 30_000

joints = load_joints(sys.argv[1])
ids = sorted(joints)
with MotorBus() as bus:
    for i in ids:
        bus.set_joint(i, joints[i])
    state = bus.enable(ids)                      # also re-references each joint to the motor's reading
    mid = {i: (joints[i].low + joints[i].high) / 2 for i in ids}
    amp = {i: 0.4 * (joints[i].high - joints[i].low) for i in ids}
    for i in ids:
        print(f"id {i}: travel {math.degrees(joints[i].travel):.0f} deg, now at {math.degrees(joints[i].angle(state[i].pos)):+.1f} deg")

    # move to mid-travel at 50,000 counts/s
    x = {i: float(state[i].pos) for i in ids}
    t_last = time.monotonic()
    while any(abs(mid[i] - x[i]) > 1 for i in ids):
        now = time.monotonic()
        for i in ids:
            step = min(50_000 * (now - t_last), abs(mid[i] - x[i]))
            x[i] += math.copysign(step, mid[i] - x[i])
            bus.command(i, pos=round(x[i]), kp=KP, kd=KD[i])
        t_last = now

    w, t0, ok = 2 * math.pi / period, time.monotonic(), True
    while ok and (t := time.monotonic() - t0) < seconds:
        fade = min(t, seconds - t, period) / period            # amplitude fades in and out over one period
        env = 0.5 - 0.5 * math.cos(math.pi * fade)
        for i in ids:
            target = mid[i] + env * amp[i] * math.sin(w * t)
            rate = env * amp[i] * w * math.cos(w * t)          # counts/s (ignores the fade's own slope)
            s = bus.command(i, pos=round(target), vel=units.pos_rate_to_vel(rate), kp=KP, kd=KD[i])
            if s and abs(s.pos - target) > MAX_LAG:
                print(f"id {i}: lag {abs(s.pos - target):.0f} counts, stopping")
                ok = False
# leaving the block returns the motors to idle, which brakes them

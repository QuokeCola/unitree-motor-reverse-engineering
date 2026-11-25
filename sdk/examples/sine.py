"""Slow sine on one joint between two positions you choose (raw counts).

    python sine.py ID LOW HIGH [PERIOD_S] [SECONDS]

NOT run through this SDK yet. The same motion was run many times with the research script
scripts/sweep.py in the parent project. Find LOW and HIGH with zero_torque.py first and stay
well inside the end stops.
"""
import math
import sys
import time

from g1_joint_motor_sdk import MotorBus, units

motor_id, low, high = int(sys.argv[1]), int(sys.argv[2]), int(sys.argv[3])
period = float(sys.argv[4]) if len(sys.argv) > 4 else 8.0
seconds = float(sys.argv[5]) if len(sys.argv) > 5 else 3 * period
KP, KD, MAX_LAG = 300, 80, 30_000
mid, amp, w = (low + high) / 2, (high - low) / 2, 2 * math.pi / period

with MotorBus() as bus:
    bus.set_limits(motor_id, low, high)
    state = bus.enable([motor_id])[motor_id]

    # move to the middle at 50,000 counts/s
    x, t_last = float(state.pos), time.monotonic()
    while abs(mid - x) > 1:
        now = time.monotonic()
        step = min(50_000 * (now - t_last), abs(mid - x))
        x, t_last = x + math.copysign(step, mid - x), now
        bus.command(motor_id, pos=round(x), kp=KP, kd=KD)

    t0 = time.monotonic()
    while (t := time.monotonic() - t0) < seconds:
        fade = min(t, seconds - t, period) / period            # amplitude fades in and out over one period
        env = 0.5 - 0.5 * math.cos(math.pi * fade)
        target = mid + env * amp * math.sin(w * t)
        rate = env * amp * w * math.cos(w * t)                 # counts/s (ignores the fade's own slope)
        s = bus.command(motor_id, pos=round(target), vel=units.pos_rate_to_vel(rate), kp=KP, kd=KD)
        if s and abs(s.pos - target) > MAX_LAG:
            print(f"lag {abs(s.pos - target):.0f} counts, stopping")
            break
# leaving the block brakes with damping, then idles the motor

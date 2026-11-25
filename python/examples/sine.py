"""!
@file sine.py
@brief Slow sine on the joints, using stored end stops.

Usage: sine.py LIMITS.json [period_s] [seconds] [port]
Record the end stops first, after every power-on, with
`g1motor limits --ids 4,5,6 --out limits.json`. Uses 80 % of each joint's
travel. Gains are those tuned on a bench wrist (ids 4, 5, 6) at 24 V.
"""
import math
import sys
import time

import g1motor

## Position gain.
KP = 300
## Largest allowed gap between target and measured position, in counts.
MAX_LAG = 30000

if len(sys.argv) < 2:
    sys.exit("usage: sine.py LIMITS.json [period_s] [seconds] [port]")
period = float(sys.argv[2]) if len(sys.argv) > 2 else 8.0
seconds = float(sys.argv[3]) if len(sys.argv) > 3 else 3 * period


def kd(motor_id: int) -> int:
    """!
    @brief Damping gain for a motor.
    @param motor_id Motor id.
    @return The gain.
    """
    return 40 if motor_id == 4 else 80


joints = g1motor.load_joints(sys.argv[1])
ids = sorted(joints)
ok = True
with g1motor.MotorBus(sys.argv[4] if len(sys.argv) > 4 else "") as bus:
    for motor_id in ids:
        bus.set_joint(motor_id, joints[motor_id])
    start = bus.enable(ids)  # also re-references each joint
    mid = {}
    amplitude = {}
    target = {}
    for motor_id in ids:
        joint = bus.joint(motor_id)
        mid[motor_id] = (joint.low + joint.high) / 2
        amplitude[motor_id] = 0.4 * (joint.high - joint.low)
        target[motor_id] = float(start[motor_id].pos)
        print(f"id {motor_id}: travel {math.degrees(joint.travel()):.0f} deg, "
              f"now at {math.degrees(joint.angle(start[motor_id].pos)):+.1f} "
              "deg")

    # Move to mid-travel at 50,000 counts/s.
    last = time.monotonic()
    moving = True
    while moving:
        now = time.monotonic()
        dt, last = now - last, now
        moving = False
        for motor_id in ids:
            gap = mid[motor_id] - target[motor_id]
            step = min(50000 * dt, abs(gap))
            target[motor_id] += math.copysign(step, gap)
            if abs(mid[motor_id] - target[motor_id]) > 1:
                moving = True
            bus.command(motor_id, round(target[motor_id]), 0, 0, KP,
                        kd(motor_id))

    omega = 2 * math.pi / period
    begin = time.monotonic()
    t = 0.0
    while ok and t < seconds:
        # The amplitude fades in and out over one period.
        fade = min(t, seconds - t, period) / period
        envelope = 0.5 - 0.5 * math.cos(math.pi * fade)
        for motor_id in ids:
            goal = (mid[motor_id] + envelope * amplitude[motor_id]
                    * math.sin(omega * t))
            rate = (envelope * amplitude[motor_id] * omega
                    * math.cos(omega * t))
            state = bus.command(motor_id, round(goal),
                                g1motor.units.pos_rate_to_vel(rate), 0, KP,
                                kd(motor_id))
            if state and abs(state.pos - goal) > MAX_LAG:
                print(f"id {motor_id}: lag {abs(state.pos - goal):.0f} "
                      "counts, stopping")
                ok = False
        t = time.monotonic() - begin
# Leaving the block returns the motors to idle.
sys.exit(0 if ok else 1)

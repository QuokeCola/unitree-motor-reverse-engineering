"""!
@file position_hold.py
@brief Holds every joint where it is with a position loop.

Gains are the ones tuned on a bench wrist (ids 4, 5, 6) at 24 V; start lower on other joints.
"""
import sys
import time

from g1_joint_motor_sdk import MotorBus

IDS = [4, 5, 6]
KP = {4: 300, 5: 300, 6: 300}
KD = {4: 40, 5: 80, 6: 80}
SECONDS = float(sys.argv[1]) if len(sys.argv) > 1 else 3.0

with MotorBus() as bus:
    start = bus.enable(IDS)
    target = {i: start[i].pos for i in IDS}
    worst = {i: 0 for i in IDS}
    end = time.monotonic() + SECONDS
    while time.monotonic() < end:
        for i in IDS:
            s = bus.command(i, pos=target[i], kp=KP[i], kd=KD[i])
            if s:
                worst[i] = max(worst[i], abs(s.pos - target[i]))
    for i in IDS:
        print(f"id {i}: held {target[i]} counts, largest error {worst[i]} counts")

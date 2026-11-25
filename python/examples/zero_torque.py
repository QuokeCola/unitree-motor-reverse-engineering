"""!
@file zero_torque.py
@brief Frees the joints for 10 s and prints how far each was moved.
"""
import time

from g1_joint_motor_sdk import MotorBus

IDS = [4, 5, 6]

with MotorBus() as bus:
    bus.enable(IDS)                      # run mode, zero torque
    low, high = {}, {}
    end = time.monotonic() + 10
    while time.monotonic() < end:
        for i in IDS:
            s = bus.command(i)           # all zero = zero torque
            if s:
                low[i], high[i] = min(low.get(i, s.pos), s.pos), max(high.get(i, s.pos), s.pos)
    for i in IDS:
        print(f"id {i}: moved between {low.get(i)} and {high.get(i)} counts")

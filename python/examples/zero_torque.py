"""!
@file zero_torque.py
@brief Frees the joints and prints how far each was moved.

Usage: zero_torque.py [seconds] [port]
"""
import sys
import time

import g1motor

seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0

with g1motor.MotorBus(sys.argv[2] if len(sys.argv) > 2 else "") as bus:
    ids = sorted(bus.scan())
    start = bus.enable(ids)  # run mode, zero torque
    low = {motor_id: start[motor_id].pos for motor_id in ids}
    high = dict(low)
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        for motor_id in ids:
            state = bus.command(motor_id)  # all zero = zero torque
            if not state:
                continue
            low[motor_id] = min(low[motor_id], state.pos)
            high[motor_id] = max(high[motor_id], state.pos)
    for motor_id in ids:
        print(f"id {motor_id}: moved between {low[motor_id]} and "
              f"{high[motor_id]} counts")

"""!
@file position_hold.py
@brief Holds every joint where it is with a position loop.

Usage: position_hold.py [seconds] [port]
Gains are those tuned on a bench wrist (ids 4, 5, 6) at 24 V.
"""
import sys
import time

import g1motor

seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 3.0

with g1motor.MotorBus(sys.argv[2] if len(sys.argv) > 2 else "") as bus:
    ids = sorted(bus.scan())
    start = bus.enable(ids)
    worst = {motor_id: 0 for motor_id in ids}
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        for motor_id in ids:
            kd = 40 if motor_id == 4 else 80
            state = bus.command(motor_id, start[motor_id].pos, 0, 0, 300, kd)
            if not state:
                continue
            worst[motor_id] = max(worst[motor_id],
                                  abs(state.pos - start[motor_id].pos))
    for motor_id in ids:
        print(f"id {motor_id}: held {start[motor_id].pos} counts, largest "
              f"error {worst[motor_id]} counts")
# Leaving the block returns the motors to idle.

"""!
@file scan.py
@brief Lists the motors on the bus. Usage: scan.py [port]
"""
import sys

import g1motor

with g1motor.MotorBus(sys.argv[1] if len(sys.argv) > 1 else "") as bus:
    found = bus.scan()
    if not found:
        print("no motors answered on ids 0-14")
    for motor_id, state in found.items():
        print(f"id {motor_id}: mode {state.mode}, position {state.pos} "
              f"counts, {state.voltage():.1f} V, coil {state.temp_coil} C, "
              f"shell {state.temp_shell} C"
              + (", FAULT" if state.fault() else ""))

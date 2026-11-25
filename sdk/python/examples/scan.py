"""List the motors on the bus."""
from g1_joint_motor_sdk import MotorBus

with MotorBus() as bus:
    for motor_id, s in bus.scan().items():
        print(f"id {motor_id}: mode {s.mode}, position {s.pos} counts, {s.voltage} V, coil {s.temp_coil} C, shell {s.temp_shell} C")

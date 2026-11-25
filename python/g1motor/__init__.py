"""!
@file __init__.py
@brief Unofficial driver for Unitree G1 joint motors over RS-485.
"""
from g1motor import units
from g1motor.joint import COUNTS_PER_ROTOR_TURN
from g1motor.joint import WRIST_GEAR_RATIOS
from g1motor.joint import Joint
from g1motor.joint import load_joints
from g1motor.motor_bus import BusBusyError
from g1motor.motor_bus import MotorBus
from g1motor.motor_bus import MotorFaultError
from g1motor.protocol import BAUD
from g1motor.protocol import BROADCAST_ID
from g1motor.protocol import CMD_LEN
from g1motor.protocol import MODE_IDLE
from g1motor.protocol import MODE_RESTART
from g1motor.protocol import MODE_RUN
from g1motor.protocol import REPLY_LEN
from g1motor.protocol import MotorCmd
from g1motor.protocol import MotorState
from g1motor.protocol import crc32_core
from g1motor.protocol import find_reply
from g1motor.serial_port import SerialPort

## Package version.
__version__ = "0.4.0"

## Names exported by `from g1motor import *`.
__all__ = [
    "BAUD", "BROADCAST_ID", "CMD_LEN", "COUNTS_PER_ROTOR_TURN", "MODE_IDLE",
    "MODE_RESTART", "MODE_RUN", "REPLY_LEN", "WRIST_GEAR_RATIOS",
    "BusBusyError", "Joint", "MotorBus", "MotorCmd", "MotorFaultError",
    "MotorState", "SerialPort", "crc32_core", "find_reply", "load_joints",
    "units",
]

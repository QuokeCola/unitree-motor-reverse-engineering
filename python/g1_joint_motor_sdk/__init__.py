"""!
@file __init__.py
@brief Unofficial driver for Unitree G1 joint motors over RS-485.
"""
from g1_joint_motor_sdk import units
from g1_joint_motor_sdk.joint import WRIST_GEAR_RATIOS
from g1_joint_motor_sdk.joint import Joint
from g1_joint_motor_sdk.joint import load_joints
from g1_joint_motor_sdk.motor_bus import BusBusyError
from g1_joint_motor_sdk.motor_bus import MotorBus
from g1_joint_motor_sdk.motor_bus import MotorFaultError
from g1_joint_motor_sdk.protocol import BAUD
from g1_joint_motor_sdk.protocol import CMD_LEN
from g1_joint_motor_sdk.protocol import MODE_IDLE
from g1_joint_motor_sdk.protocol import MODE_RESTART
from g1_joint_motor_sdk.protocol import MODE_RUN
from g1_joint_motor_sdk.protocol import REPLY_LEN
from g1_joint_motor_sdk.protocol import MotorCmd
from g1_joint_motor_sdk.protocol import MotorState
from g1_joint_motor_sdk.protocol import crc32_core
from g1_joint_motor_sdk.protocol import find_reply

## Package version.
__version__ = "0.3.0"

## Names exported by `from g1_joint_motor_sdk import *`.
__all__ = [
    "BAUD", "CMD_LEN", "MODE_IDLE", "MODE_RESTART", "MODE_RUN", "REPLY_LEN",
    "WRIST_GEAR_RATIOS", "BusBusyError", "Joint", "MotorBus", "MotorCmd",
    "MotorFaultError", "MotorState", "crc32_core", "find_reply", "load_joints",
    "units",
]

"""Unofficial driver for Unitree G1 joint motors over RS-485. See README.md."""
from . import units
from .bus import BusBusyError, MotorBus, MotorFaultError
from .protocol import (BAUD, CMD_LEN, MODE_IDLE, MODE_RUN, REPLY_LEN, MotorCmd, MotorState, crc32_core,
                       find_reply)

__version__ = "0.1.0"
__all__ = ["MotorBus", "BusBusyError", "MotorFaultError", "MotorCmd", "MotorState", "crc32_core", "find_reply", "units",
           "BAUD", "CMD_LEN", "REPLY_LEN", "MODE_IDLE", "MODE_RUN"]

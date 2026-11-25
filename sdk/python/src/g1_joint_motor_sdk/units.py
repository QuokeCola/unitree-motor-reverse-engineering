"""Conversion between raw counts and physical units.

Scales follow Unitree's public GO-M8010-6 protocol. How far each is confirmed on G1 motors:
- position, 32768 counts per rotor turn: supported. After a restart the reading jumps by exactly
  524,288 (16 x 32768) on one motor and 819,200 (25 x 32768) on another, i.e. whole output turns
  of 16:1 and 25:1 gears.
- velocity: follows from the position scale and the measured ratio of 128 position counts per
  second per velocity count.
- torque, 1/256 N*m per count: ASSUMED, not checked.
All values refer to the ROTOR, before the gearbox. Use joints.Joint for output-shaft angles.
"""
import math

POS_RAD_PER_COUNT = 2 * math.pi / 32768      # rad
VEL_RADS_PER_COUNT = 2 * math.pi / 256       # rad/s
TAU_NM_PER_COUNT = 1 / 256                   # N*m
POS_COUNTS_PER_SEC_PER_VEL_COUNT = 128.0     # exact consequence of the two scales above


def pos_to_rad(counts: float) -> float:
    return counts * POS_RAD_PER_COUNT


def rad_to_pos(rad: float) -> int:
    return round(rad / POS_RAD_PER_COUNT)


def vel_to_rads(counts: float) -> float:
    return counts * VEL_RADS_PER_COUNT


def rads_to_vel(rads: float) -> int:
    return round(rads / VEL_RADS_PER_COUNT)


def pos_rate_to_vel(counts_per_second: float) -> int:
    """Velocity command (raw) for a target moving at this many position counts per second."""
    return round(counts_per_second / POS_COUNTS_PER_SEC_PER_VEL_COUNT)


def tau_to_nm(counts: float) -> float:
    return counts * TAU_NM_PER_COUNT


def nm_to_tau(nm: float) -> int:
    return round(nm / TAU_NM_PER_COUNT)

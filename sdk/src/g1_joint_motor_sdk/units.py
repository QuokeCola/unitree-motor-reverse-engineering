"""Conversion between raw counts and physical units.

ASSUMED scales, taken from Unitree's public GO-M8010-6 protocol. Only the ratio between
the position and velocity scales has been checked against G1 data (measured 114-124
against the expected 128). All values refer to the ROTOR, before the gearbox; the gear
ratios of the G1 joints are not known here.
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

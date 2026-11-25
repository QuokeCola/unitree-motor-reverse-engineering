"""!
@file units.py
@brief Conversion between raw counts and physical units, on the rotor side.

Position (32768 counts per rotor turn) is supported by data: after a restart
the reading jumps by exactly 16 or 25 rotor turns. Velocity follows from it
and the measured ratio of 128. The torque scale is assumed from the public
GO-M8010-6 protocol and not checked. Use joint.Joint for output-shaft angles.
"""
import math

## Radians of rotor angle per position count.
POS_RAD_PER_COUNT = 2 * math.pi / 32768
## Radians per second of rotor speed per velocity count.
VEL_RADS_PER_COUNT = 2 * math.pi / 256
## Newton metres of rotor torque per torque count (assumed).
TAU_NM_PER_COUNT = 1 / 256
## Position counts per second represented by one velocity count.
POS_COUNTS_PER_VEL_COUNT = 128.0


def pos_to_rad(counts: float) -> float:
    """!
    @brief Converts a position to rotor angle.
    @param counts Position in counts.
    @return Angle in rad.
    """
    return counts * POS_RAD_PER_COUNT


def rad_to_pos(rad: float) -> int:
    """!
    @brief Converts a rotor angle to a position.
    @param rad Angle in rad.
    @return Position in counts.
    """
    return round(rad / POS_RAD_PER_COUNT)


def vel_to_rads(counts: float) -> float:
    """!
    @brief Converts a velocity reading to rotor speed.
    @param counts Velocity in counts.
    @return Speed in rad/s.
    """
    return counts * VEL_RADS_PER_COUNT


def rads_to_vel(rads: float) -> int:
    """!
    @brief Converts a rotor speed to a velocity command.
    @param rads Speed in rad/s.
    @return Velocity in counts.
    """
    return round(rads / VEL_RADS_PER_COUNT)


def pos_rate_to_vel(counts_per_second: float) -> int:
    """!
    @brief Velocity command for a position target that moves at a given rate.
    @param counts_per_second Rate of the position target.
    @return Velocity in counts.
    """
    return round(counts_per_second / POS_COUNTS_PER_VEL_COUNT)


def tau_to_nm(counts: float) -> float:
    """!
    @brief Converts a torque reading to rotor torque (assumed scale).
    @param counts Torque in counts.
    @return Torque in N*m.
    """
    return counts * TAU_NM_PER_COUNT


def nm_to_tau(nm: float) -> int:
    """!
    @brief Converts a rotor torque to a torque command (assumed scale).
    @param nm Torque in N*m.
    @return Torque in counts.
    """
    return round(nm / TAU_NM_PER_COUNT)

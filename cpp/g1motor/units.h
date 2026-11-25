/// @file units.h
/// @brief Conversion between raw counts and physical units, on the rotor side.
///
/// Position (32768 counts per rotor turn) is supported by data: after a
/// restart the reading jumps by exactly 16 or 25 rotor turns. Velocity follows
/// from it and the measured ratio of 128. The torque scale is assumed from the
/// public GO-M8010-6 protocol and not checked. Use Joint for output angles.

#ifndef G1MOTOR_UNITS_H_
#define G1MOTOR_UNITS_H_

#include <cmath>

namespace g1motor {
namespace units {

/// Radians of rotor angle per position count.
inline constexpr double POS_RAD_PER_COUNT =
    2.0 * 3.14159265358979323846 / 32768;
/// Radians per second of rotor speed per velocity count.
inline constexpr double VEL_RADS_PER_COUNT = 2.0 * 3.14159265358979323846 / 256;
/// Newton metres of rotor torque per torque count (assumed).
inline constexpr double TAU_NM_PER_COUNT = 1.0 / 256;
/// Position counts per second represented by one velocity count.
inline constexpr double POS_COUNTS_PER_VEL_COUNT = 128.0;

/// @brief Converts a position to rotor angle.
/// @param counts Position in counts.
/// @return Angle in rad.
inline double pos_to_rad(double counts) { return counts * POS_RAD_PER_COUNT; }

/// @brief Converts a rotor angle to a position.
/// @param rad Angle in rad.
/// @return Position in counts.
inline long rad_to_pos(double rad) {
  return std::lround(rad / POS_RAD_PER_COUNT);
}

/// @brief Converts a velocity reading to rotor speed.
/// @param counts Velocity in counts.
/// @return Speed in rad/s.
inline double vel_to_rads(double counts) { return counts * VEL_RADS_PER_COUNT; }

/// @brief Converts a rotor speed to a velocity command.
/// @param rads Speed in rad/s.
/// @return Velocity in counts.
inline long rads_to_vel(double rads) {
  return std::lround(rads / VEL_RADS_PER_COUNT);
}

/// @brief Velocity command for a position target that moves at a given rate.
/// @param counts_per_second Rate of the position target.
/// @return Velocity in counts.
inline long pos_rate_to_vel(double counts_per_second) {
  return std::lround(counts_per_second / POS_COUNTS_PER_VEL_COUNT);
}

/// @brief Converts a torque reading to rotor torque (assumed scale).
/// @param counts Torque in counts.
/// @return Torque in N*m.
inline double tau_to_nm(double counts) { return counts * TAU_NM_PER_COUNT; }

/// @brief Converts a rotor torque to a torque command (assumed scale).
/// @param nm Torque in N*m.
/// @return Torque in counts.
inline long nm_to_tau(double nm) { return std::lround(nm / TAU_NM_PER_COUNT); }

}  // namespace units
}  // namespace g1motor

#endif  // G1MOTOR_UNITS_H_

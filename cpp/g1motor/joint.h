/// @file joint.h
/// @brief Joint-level view of a motor: gear ratio, end stops, software zero.

#ifndef G1MOTOR_JOINT_H_
#define G1MOTOR_JOINT_H_

#include <cstdint>
#include <map>
#include <optional>
#include <string>

namespace g1motor {

/// Position counts in one turn of the motor rotor.
inline constexpr int COUNTS_PER_ROTOR_TURN = 32768;
/// Position counts per second represented by one velocity count.
inline constexpr double POS_COUNTS_PER_VEL_COUNT = 128.0;
/// Gear ratios seen on one G1 wrist (ids 4, 5, 6); id 5 is assumed.
inline const std::map<int, double> WRIST_GEAR_RATIOS = {
    {4, 16.0}, {5, 25.0}, {6, 25.0}};

/// @brief Gear ratio, end stops and zero of one joint, in raw counts.
///
/// After a power cycle or restart a motor's position reading can come back
/// shifted by a whole number of output-shaft turns. sync() moves the stored
/// zero and end stops by the same amount.
struct Joint {
  double gear_ratio = 1.0;           ///< Rotor turns per output turn.
  std::optional<std::int64_t> low;   ///< Lower end stop.
  std::optional<std::int64_t> high;  ///< Upper end stop.
  std::int64_t zero = 0;             ///< Count that is angle 0.
  std::int64_t margin = 0;           ///< Kept clear of each stop by clamp().
  std::int64_t tolerance = 3000;     ///< Slack allowed by sync().

  Joint() = default;

  /// @brief Joint without end stops.
  /// @param gear_ratio Rotor turns per output turn.
  explicit Joint(double gear_ratio);

  /// @brief Joint with end stops; the zero is the middle of the travel.
  /// @param gear_ratio Rotor turns per output turn.
  /// @param low Lower end stop in counts.
  /// @param high Upper end stop in counts.
  /// @throws std::invalid_argument if low is not below high.
  Joint(double gear_ratio, std::int64_t low, std::int64_t high);

  /// @brief Size of one output turn.
  /// @return Position counts in one turn of the output shaft.
  std::int64_t turn_counts() const;

  /// @brief Angle between the end stops.
  /// @return Travel in rad, or empty without end stops.
  std::optional<double> travel() const;

  /// @brief Shifts zero and end stops by whole output turns to match a
  ///        position reading.
  ///
  /// Call after every power-on or restart.
  /// @param pos Current position reading.
  /// @return Number of output turns shifted.
  /// @throws std::invalid_argument if the reading still misses the end stops.
  int sync(std::int64_t pos);

  /// @brief Converts a position to an output-shaft angle.
  /// @param pos Position in counts.
  /// @return Angle in rad from the zero.
  double angle(double pos) const;

  /// @brief Converts an output-shaft angle to a position.
  /// @param angle Angle in rad from the zero.
  /// @return Position in counts.
  std::int32_t counts(double angle) const;

  /// @brief Converts a velocity reading to output-shaft speed.
  /// @param vel Velocity in counts.
  /// @return Speed in rad/s.
  double speed(double vel) const;

  /// @brief Limits a position to the end stops less the margin.
  /// @param pos Position in counts.
  /// @return The limited position.
  std::int32_t clamp(double pos) const;
};

/// @brief Reads joints from an end-stop file written by `g1motor limits`.
/// @param path JSON file of the form {"4": [low, high], ...}.
/// @param gear_ratios Gear ratio per id.
/// @param margin Margin given to every joint.
/// @return Joints by motor id.
/// @throws std::runtime_error if the file cannot be read or parsed, or an id
///     has no gear ratio.
std::map<int, Joint> load_joints(
    const std::string& path,
    const std::map<int, double>& gear_ratios = WRIST_GEAR_RATIOS,
    std::int64_t margin = 0);

}  // namespace g1motor

#endif  // G1MOTOR_JOINT_H_

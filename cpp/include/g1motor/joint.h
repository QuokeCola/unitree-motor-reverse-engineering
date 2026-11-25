/// @file joint.h
/// @brief Joint-level view of a motor: gear ratio, end stops, software zero.

#ifndef G1MOTOR_JOINT_H_
#define G1MOTOR_JOINT_H_

#include <cmath>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>

namespace g1motor {

/// Position counts in one turn of the motor rotor.
inline constexpr int kCountsPerRotorTurn = 32768;
/// Position counts per second represented by one velocity count.
inline constexpr double kPosCountsPerVelCount = 128.0;

/// @brief Gear ratio, end stops and zero of one joint, in raw counts.
///
/// After a power cycle or restart a motor's position reading can come back
/// shifted by a whole number of output-shaft turns. Sync() moves the stored
/// zero and end stops by the same amount.
struct Joint {
  double gear_ratio = 1.0;           ///< Rotor turns per output turn.
  std::optional<std::int64_t> low;   ///< Lower end stop.
  std::optional<std::int64_t> high;  ///< Upper end stop.
  std::int64_t zero = 0;             ///< Count that is angle 0.
  std::int64_t margin = 0;           ///< Kept clear of each stop by Clamp().
  std::int64_t tolerance = 3000;     ///< Slack allowed by Sync().

  Joint() = default;

  /// @brief Joint without end stops.
  explicit Joint(double ratio) : gear_ratio(ratio) {}

  /// @brief Joint with end stops; the zero is the middle of the travel.
  /// @throws std::invalid_argument if lo is not below hi.
  Joint(double ratio, std::int64_t lo, std::int64_t hi)
      : gear_ratio(ratio), low(lo), high(hi), zero((lo + hi) / 2) {
    if (lo >= hi) throw std::invalid_argument("low must be below high");
  }

  /// @return Position counts in one turn of the output shaft.
  std::int64_t turn_counts() const {
    return std::llround(gear_ratio * kCountsPerRotorTurn);
  }

  /// @brief Shifts zero and end stops by whole output turns to match a reading.
  ///
  /// Call after every power-on or restart.
  /// @param pos Current position reading.
  /// @return Number of output turns shifted.
  /// @throws std::runtime_error if the reading still misses the end stops.
  int Sync(std::int64_t pos) {
    const double reference =
        low ? (*low + *high) / 2.0 : static_cast<double>(zero);
    const int turns = static_cast<int>(
        std::llround((pos - reference) / static_cast<double>(turn_counts())));
    const std::int64_t shift = turns * turn_counts();
    zero += shift;
    if (low) {
      low = *low + shift;
      high = *high + shift;
      if (pos < *low - tolerance || pos > *high + tolerance) {
        throw std::runtime_error("position " + std::to_string(pos) +
                                 " does not fit the end stops");
      }
    }
    return turns;
  }

  /// @return Output-shaft angle in rad for a position in counts.
  double Angle(double pos) const {
    return (pos - zero) / turn_counts() * 2.0 * M_PI;
  }

  /// @return Position in counts for an output-shaft angle in rad.
  std::int32_t Counts(double angle_rad) const {
    return static_cast<std::int32_t>(
        std::llround(zero + angle_rad / (2.0 * M_PI) * turn_counts()));
  }

  /// @return Output-shaft speed in rad/s for a raw velocity reading.
  double Speed(double vel) const {
    return vel * kPosCountsPerVelCount / turn_counts() * 2.0 * M_PI;
  }

  /// @return The position limited to the end stops less the margin.
  std::int32_t Clamp(double pos) const {
    if (!low) return static_cast<std::int32_t>(std::llround(pos));
    const double lo = static_cast<double>(*low + margin);
    const double hi = static_cast<double>(*high - margin);
    return static_cast<std::int32_t>(std::llround(pos < lo   ? lo
                                                  : pos > hi ? hi
                                                             : pos));
  }
};

}  // namespace g1motor

#endif  // G1MOTOR_JOINT_H_

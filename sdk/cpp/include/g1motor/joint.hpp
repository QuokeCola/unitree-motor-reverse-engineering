// Joint-level view of a motor: gear ratio, end stops, a software zero, and the whole-turn
// ambiguity of the position reading after a power cycle or restart.
#pragma once

#include <cmath>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>

namespace g1motor {

constexpr int kCountsPerRotorTurn = 32768;

struct Joint {
    double gear_ratio = 1.0;
    std::optional<std::int64_t> low, high;   // end stops, raw counts
    std::int64_t zero = 0;                   // raw count that is angle 0
    std::int64_t margin = 0;                 // counts kept clear of each end stop by clamp()
    std::int64_t tolerance = 3000;           // how far outside the end stops a reading may be in sync()

    Joint() = default;
    explicit Joint(double ratio) : gear_ratio(ratio) {}
    // Zero defaults to the middle of the travel.
    Joint(double ratio, std::int64_t lo, std::int64_t hi) : gear_ratio(ratio), low(lo), high(hi), zero((lo + hi) / 2) {
        if (lo >= hi) throw std::invalid_argument("low must be below high");
    }

    std::int64_t turn_counts() const { return std::llround(gear_ratio * kCountsPerRotorTurn); }

    // Shift zero and end stops by whole output turns to match a position reading. Call after every
    // power-on or restart. Returns the turns shifted; throws if the reading still does not fit.
    int sync(std::int64_t pos) {
        const double reference = low ? (*low + *high) / 2.0 : static_cast<double>(zero);
        const auto k = static_cast<int>(std::llround((pos - reference) / static_cast<double>(turn_counts())));
        const std::int64_t shift = k * turn_counts();
        zero += shift;
        if (low) {
            low = *low + shift;
            high = *high + shift;
            if (pos < *low - tolerance || pos > *high + tolerance)
                throw std::runtime_error("position " + std::to_string(pos) + " does not fit the end stops; re-measure them");
        }
        return k;
    }

    double angle(double pos) const { return (pos - zero) / turn_counts() * 2.0 * M_PI; }                  // rad, output shaft
    std::int32_t counts(double angle_rad) const { return static_cast<std::int32_t>(std::llround(zero + angle_rad / (2.0 * M_PI) * turn_counts())); }
    double speed(double vel) const { return vel * 128.0 / turn_counts() * 2.0 * M_PI; }                   // rad/s, output shaft
    std::int32_t clamp(double pos) const {
        if (!low) return static_cast<std::int32_t>(std::llround(pos));
        const double lo = static_cast<double>(*low + margin), hi = static_cast<double>(*high - margin);
        return static_cast<std::int32_t>(std::llround(pos < lo ? lo : pos > hi ? hi : pos));
    }
};

}  // namespace g1motor

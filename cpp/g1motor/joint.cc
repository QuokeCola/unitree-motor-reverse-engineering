#include "g1motor/joint.h"

#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace g1motor {
namespace {

/// Value of pi.
constexpr double PI = 3.14159265358979323846;

}  // namespace

Joint::Joint(double gear_ratio) : gear_ratio(gear_ratio) {}

Joint::Joint(double gear_ratio, std::int64_t low, std::int64_t high)
    : gear_ratio(gear_ratio), low(low), high(high), zero((low + high) / 2) {
  if (low >= high) throw std::invalid_argument("low must be below high");
}

std::int64_t Joint::turn_counts() const {
  return std::llround(gear_ratio * COUNTS_PER_ROTOR_TURN);
}

std::optional<double> Joint::travel() const {
  if (!low) return std::nullopt;
  return static_cast<double>(*high - *low) / turn_counts() * 2.0 * PI;
}

int Joint::sync(std::int64_t pos) {
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
      throw std::invalid_argument("position " + std::to_string(pos) +
                                  " does not fit the end stops");
    }
  }
  return turns;
}

double Joint::angle(double pos) const {
  return (pos - zero) / turn_counts() * 2.0 * PI;
}

std::int32_t Joint::counts(double angle) const {
  return static_cast<std::int32_t>(
      std::llround(zero + angle / (2.0 * PI) * turn_counts()));
}

double Joint::speed(double vel) const {
  return vel * POS_COUNTS_PER_VEL_COUNT / turn_counts() * 2.0 * PI;
}

std::int32_t Joint::clamp(double pos) const {
  if (!low) return static_cast<std::int32_t>(std::llround(pos));
  const double lowest = static_cast<double>(*low + margin);
  const double highest = static_cast<double>(*high - margin);
  return static_cast<std::int32_t>(std::llround(pos < lowest    ? lowest
                                                : pos > highest ? highest
                                                                : pos));
}

std::map<int, Joint> load_joints(const std::string& path,
                                 const std::map<int, double>& gear_ratios,
                                 std::int64_t margin) {
  std::ifstream file(path);
  if (!file) throw std::runtime_error("cannot open " + path);
  std::stringstream content;
  content << file.rdbuf();
  const std::string text = content.str();

  // The file is a flat object: {"4": [low, high], "5": [low, high]}.
  std::map<int, Joint> joints;
  std::size_t at = 0;
  while ((at = text.find('"', at)) != std::string::npos) {
    const std::size_t key_end = text.find('"', at + 1);
    const std::size_t open = text.find('[', key_end);
    const std::size_t comma = text.find(',', open);
    const std::size_t close = text.find(']', comma);
    if (key_end == std::string::npos || open == std::string::npos ||
        comma == std::string::npos || close == std::string::npos) {
      throw std::runtime_error("cannot parse " + path);
    }
    const int id = std::stoi(text.substr(at + 1, key_end - at - 1));
    const auto low = std::stoll(text.substr(open + 1, comma - open - 1));
    const auto high = std::stoll(text.substr(comma + 1, close - comma - 1));
    const auto ratio = gear_ratios.find(id);
    if (ratio == gear_ratios.end()) {
      throw std::runtime_error("no gear ratio for motor " + std::to_string(id));
    }
    Joint joint(ratio->second, low, high);
    joint.margin = margin;
    joints[id] = joint;
    at = close + 1;
  }
  return joints;
}

}  // namespace g1motor

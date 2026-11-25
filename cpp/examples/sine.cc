/// @file sine.cc
/// @brief Slow sine on the joints, using stored end stops.
///
/// Usage: sine LIMITS.json [period_s] [seconds] [port]
/// Record the end stops first, after every power-on, with
/// `g1motor limits --ids 4,5,6 --out limits.json`. Uses 80 % of each joint's
/// travel. Gains are those tuned on a bench wrist (ids 4, 5, 6) at 24 V.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <map>
#include <vector>

#include "g1motor/g1motor.h"

namespace {

/// Value of pi.
constexpr double PI = 3.14159265358979323846;
/// Position gain.
constexpr int KP = 300;
/// Largest allowed gap between target and measured position, in counts.
constexpr double MAX_LAG = 30000;

/// @return Seconds elapsed since `start`.
double seconds_since(std::chrono::steady_clock::time_point start) {
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - start)
      .count();
}

}  // namespace

/// Moves every joint in the limits file through a sine and back to idle.
int main(int argc, char** argv) {
  if (argc < 2) {
    std::fprintf(stderr,
                 "usage: sine LIMITS.json [period_s] [seconds] [port]\n");
    return 2;
  }
  const double period = argc > 2 ? std::atof(argv[2]) : 8.0;
  const double seconds = argc > 3 ? std::atof(argv[3]) : 3 * period;
  try {
    auto joints = g1motor::load_joints(argv[1]);
    std::vector<int> ids;
    g1motor::MotorBus bus(argc > 4 ? argv[4] : "");
    for (const auto& [id, joint] : joints) {
      ids.push_back(id);
      bus.set_joint(id, joint);
    }
    const auto start = bus.enable(ids);  // also re-references each joint
    std::map<int, double> mid;
    std::map<int, double> amplitude;
    std::map<int, double> target;
    for (int id : ids) {
      const g1motor::Joint& joint = bus.joint(id);
      mid[id] = (*joint.low + *joint.high) / 2.0;
      amplitude[id] = 0.4 * (*joint.high - *joint.low);
      target[id] = start.at(id).pos;
      std::printf("id %d: travel %.0f deg, now at %+.1f deg\n", id,
                  *joint.travel() * 180 / PI,
                  joint.angle(start.at(id).pos) * 180 / PI);
    }
    auto kd = [](int id) { return id == 4 ? 40 : 80; };

    // Move to mid-travel at 50,000 counts/s.
    auto last = std::chrono::steady_clock::now();
    bool moving = true;
    while (moving) {
      const double dt = seconds_since(last);
      last = std::chrono::steady_clock::now();
      moving = false;
      for (int id : ids) {
        const double gap = mid[id] - target[id];
        const double step = std::min(50000 * dt, std::fabs(gap));
        target[id] += std::copysign(step, gap);
        if (std::fabs(mid[id] - target[id]) > 1) moving = true;
        bus.command(id, std::lround(target[id]), 0, 0, KP, kd(id));
      }
    }

    const double omega = 2 * PI / period;
    const auto begin = std::chrono::steady_clock::now();
    bool ok = true;
    for (double t = 0; ok && t < seconds; t = seconds_since(begin)) {
      // The amplitude fades in and out over one period.
      const double fade = std::min({t, seconds - t, period}) / period;
      const double envelope = 0.5 - 0.5 * std::cos(PI * fade);
      for (int id : ids) {
        const double goal =
            mid[id] + envelope * amplitude[id] * std::sin(omega * t);
        const double rate =
            envelope * amplitude[id] * omega * std::cos(omega * t);
        const auto state =
            bus.command(id, std::lround(goal),
                        static_cast<int>(g1motor::units::pos_rate_to_vel(rate)),
                        0, KP, kd(id));
        if (state && std::fabs(state->pos - goal) > MAX_LAG) {
          std::printf("id %d: lag %.0f counts, stopping\n", id,
                      std::fabs(state->pos - goal));
          ok = false;
        }
      }
    }
    return ok ? 0 : 1;  // The destructor returns the motors to idle.
  } catch (const std::exception& e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 2;
  }
}

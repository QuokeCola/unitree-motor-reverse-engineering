/// @file zero_torque.cc
/// @brief Frees the joints and prints how far each was moved.
///
/// Usage: zero_torque [seconds] [port]

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <map>
#include <vector>

#include "g1motor/g1motor.h"

/// Enables every motor found with zero torque and records its travel.
int main(int argc, char** argv) {
  const double seconds = argc > 1 ? std::atof(argv[1]) : 10.0;
  try {
    g1motor::MotorBus bus(argc > 2 ? argv[2] : "");
    std::vector<int> ids;
    for (const auto& [id, state] : bus.scan()) ids.push_back(id);
    const auto start = bus.enable(ids);  // run mode, zero torque
    std::map<int, int> low;
    std::map<int, int> high;
    for (int id : ids) low[id] = high[id] = start.at(id).pos;
    const auto begin = std::chrono::steady_clock::now();
    while (
        std::chrono::duration<double>(std::chrono::steady_clock::now() - begin)
            .count() < seconds) {
      for (int id : ids) {
        const auto state = bus.command(id);  // all zero = zero torque
        if (!state) continue;
        low[id] = std::min(low[id], state->pos);
        high[id] = std::max(high[id], state->pos);
      }
    }
    for (int id : ids) {
      std::printf("id %d: moved between %d and %d counts\n", id, low[id],
                  high[id]);
    }
    return 0;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 2;
  }
}

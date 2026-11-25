/// @file position_hold.cc
/// @brief Holds every joint where it is with a position loop.
///
/// Usage: position_hold [seconds] [port]
/// Gains are those tuned on a bench wrist (ids 4, 5, 6) at 24 V.

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <map>
#include <vector>

#include "g1motor/g1motor.h"

/// Holds every motor found at its start position and prints the error.
int main(int argc, char** argv) {
  const double seconds = argc > 1 ? std::atof(argv[1]) : 3.0;
  try {
    g1motor::MotorBus bus(argc > 2 ? argv[2] : "");
    std::vector<int> ids;
    for (const auto& [id, state] : bus.scan()) ids.push_back(id);
    const auto start = bus.enable(ids);
    std::map<int, int> worst;
    const auto begin = std::chrono::steady_clock::now();
    while (
        std::chrono::duration<double>(std::chrono::steady_clock::now() - begin)
            .count() < seconds) {
      for (int id : ids) {
        const int kd = id == 4 ? 40 : 80;
        const auto state = bus.command(id, start.at(id).pos, 0, 0, 300, kd);
        if (!state) continue;
        worst[id] =
            std::max(worst[id], std::abs(state->pos - start.at(id).pos));
      }
    }
    for (int id : ids) {
      std::printf("id %d: held %d counts, largest error %d counts\n", id,
                  start.at(id).pos, worst[id]);
    }
    return 0;  // The destructor returns the motors to idle.
  } catch (const std::exception& e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 2;
  }
}

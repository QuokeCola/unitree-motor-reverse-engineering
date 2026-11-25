/// @file position_hold.cc
/// @brief Holds every motor found where it is.
///
/// Usage: g1motor_position_hold [seconds] [port]
/// Gains are those tuned on a bench wrist (ids 4, 5, 6) at 24 V.

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <map>
#include <vector>

#include "g1motor/g1motor.h"

int main(int argc, char** argv) {
  const double seconds = argc > 1 ? std::atof(argv[1]) : 3.0;
  try {
    g1motor::MotorBus bus(argc > 2 ? argv[2] : "");
    std::vector<int> ids;
    for (const auto& [id, state] : bus.Scan()) ids.push_back(id);
    if (ids.empty()) {
      std::puts("no motors found");
      return 1;
    }
    const auto start = bus.Enable(ids);
    std::map<int, int> worst_error;
    long cycles = 0;
    const auto begin = std::chrono::steady_clock::now();
    auto elapsed = [&begin] {
      return std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                           begin)
          .count();
    };
    while (elapsed() < seconds) {
      for (int id : ids) {
        const int kd = id == 4 ? 40 : 80;
        const auto state = bus.Command(id, start.at(id).pos, 0, 0, 300, kd);
        if (!state) continue;
        const int error = std::abs(state->pos - start.at(id).pos);
        if (error > worst_error[id]) worst_error[id] = error;
      }
      ++cycles;
    }
    for (int id : ids) {
      std::printf("id %d: held %d counts, largest error %d counts\n", id,
                  start.at(id).pos, worst_error[id]);
    }
    std::printf("%.0f exchanges per second per motor\n", cycles / seconds);
    return 0;  // The destructor returns the motors to idle.
  } catch (const std::exception& e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 2;
  }
}

/// @file scan.cc
/// @brief Lists the motors on the bus. Usage: scan [port]

#include <cstdio>
#include <exception>

#include "g1motor/g1motor.h"

/// Scans the bus and prints one line per motor.
int main(int argc, char** argv) {
  try {
    g1motor::MotorBus bus(argc > 1 ? argv[1] : "");
    const auto found = bus.scan();
    if (found.empty()) std::puts("no motors answered on ids 0-14");
    for (const auto& [id, state] : found) {
      std::printf(
          "id %d: mode %d, position %d counts, %.1f V, coil %d C, shell %d "
          "C%s\n",
          id, state.mode, state.pos, state.voltage(), state.temp_coil,
          state.temp_shell, state.fault() ? ", FAULT" : "");
    }
    return found.empty() ? 1 : 0;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 2;
  }
}

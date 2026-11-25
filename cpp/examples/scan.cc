/// @file scan.cc
/// @brief Lists the motors on the bus. Usage: g1motor_scan [port]

#include <cstdio>
#include <exception>

#include "g1motor/g1motor.h"

int main(int argc, char** argv) {
  try {
    g1motor::MotorBus bus(argc > 1 ? argv[1] : "");
    const auto found = bus.Scan();
    if (found.empty()) std::puts("no motors answered on ids 0-14");
    for (const auto& [id, state] : found) {
      std::printf(
          "id %d: mode %d  pos %9d  vel %6d  tau %5d  %4.1f V  coil %d C  "
          "shell %d C%s\n",
          id, state.mode, state.pos, state.vel, state.tau, state.voltage(),
          state.temp_coil, state.temp_shell, state.fault() ? "  FAULT" : "");
    }
    return found.empty() ? 1 : 0;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 2;
  }
}

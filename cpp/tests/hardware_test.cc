/// @file hardware_test.cc
/// @brief Live test on motors with no controller on the bus.
///
/// Usage: hardware_test [port]
/// Scans, enables every motor found, holds position for 2 s, restarts one
/// motor and checks it comes back. The joints are held in place, not moved.

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <exception>
#include <vector>

#include "g1motor/g1motor.h"
#include "test_util.h"

/// Runs the live test and returns 1 if any check failed.
int main(int argc, char** argv) {
  try {
    g1motor::MotorBus bus(argc > 1 ? argv[1] : "");

    const auto found = bus.scan();
    CHECK(!found.empty());
    if (found.empty()) return g1motor_test::finish("hardware_test");
    std::vector<int> ids;
    bool any_fault = false;
    for (const auto& [id, state] : found) {
      ids.push_back(id);
      CHECK(state.mode == g1motor::MODE_IDLE);
      CHECK(state.voltage() > 10.0 && state.voltage() < 60.0);
      CHECK(state.temp_coil >= state.temp_shell);
      any_fault = any_fault || state.fault();
      std::printf("id %d: pos %d, %.1f V, status 0x%x\n", id, state.pos,
                  state.voltage(), state.error);
    }
    if (any_fault) bus.restart(ids);

    g1motor::MotorCmd absent;
    absent.id = (ids.front() + 7) % 15;
    if (found.count(absent.id) == 0) CHECK(!bus.transfer(absent));

    const auto start = bus.enable(ids);
    for (int id : ids) CHECK(start.at(id).mode == g1motor::MODE_RUN);

    int worst_error = 0;
    long cycles = 0;
    const auto begin = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - begin < std::chrono::seconds(2)) {
      for (int id : ids) {
        const auto state =
            bus.command(id, start.at(id).pos, 0, 0, 300, id == 4 ? 40 : 80);
        CHECK(state.has_value());
        if (!state) continue;
        worst_error =
            std::max(worst_error, std::abs(state->pos - start.at(id).pos));
      }
      ++cycles;
    }
    std::printf("hold: largest error %d counts, %ld exchanges/s per motor\n",
                worst_error, cycles / 2);
    CHECK(worst_error < 600);
    CHECK(cycles / 2 > 100);

    bus.disable();
    const auto idle = bus.read(ids.front());
    CHECK(idle && idle->mode == g1motor::MODE_IDLE);

    const auto after = bus.restart({ids.front()});
    CHECK(after.count(ids.front()) == 1);
    if (!after.empty()) {
      const auto& state = after.begin()->second;
      CHECK(state.mode == g1motor::MODE_IDLE && !state.fault());
    }
  } catch (const std::exception& e) {
    std::printf("exception: %s\n", e.what());
    ++g1motor_test::g_failures;
  }
  return g1motor_test::finish("hardware_test");
}

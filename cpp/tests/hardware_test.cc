/// @file hardware_test.cc
/// @brief Live test on motors with no controller on the bus.
///
/// Usage: g1motor_hardware_test [port]
/// Scans, enables every motor found, holds position for 2 s, restarts one
/// motor and checks it comes back. The joints are held in place, not moved.

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

    const auto found = bus.Scan();
    CHECK(!found.empty());
    if (found.empty()) return g1motor_test::Finish("hardware_test");
    std::vector<int> ids;
    for (const auto& [id, state] : found) {
      ids.push_back(id);
      CHECK(state.mode == g1motor::kIdle);
      CHECK(state.voltage() > 10.0 && state.voltage() < 60.0);
      CHECK(state.temp_coil >= state.temp_shell);
      std::printf("id %d: pos %d, %.1f V, status 0x%x\n", id, state.pos,
                  state.voltage(), state.error);
    }
    if (found.begin()->second.fault()) bus.Restart(ids);

    // A motor that is not on the bus must stay silent.
    g1motor::MotorCmd absent;
    absent.id = (ids.front() + 7) % g1motor::kBroadcastId;
    if (found.count(absent.id) == 0) CHECK(!bus.Transfer(absent));

    const auto start = bus.Enable(ids);
    for (int id : ids) CHECK(start.at(id).mode == g1motor::kRun);

    int worst_error = 0;
    long cycles = 0;
    const auto begin = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - begin < std::chrono::seconds(2)) {
      for (int id : ids) {
        const auto state =
            bus.Command(id, start.at(id).pos, 0, 0, 300, id == 4 ? 40 : 80);
        CHECK(state.has_value());
        if (!state) continue;
        const int error = std::abs(state->pos - start.at(id).pos);
        if (error > worst_error) worst_error = error;
      }
      ++cycles;
    }
    std::printf("hold: largest error %d counts, %ld exchanges/s per motor\n",
                worst_error, cycles / 2);
    CHECK(worst_error < 600);
    CHECK(cycles / 2 > 100);

    bus.Disable();
    const auto idle = bus.Read(ids.front());
    CHECK(idle && idle->mode == g1motor::kIdle);

    const auto after = bus.Restart({ids.front()});
    CHECK(after.count(ids.front()) == 1);
    if (!after.empty()) {
      const auto& state = after.begin()->second;
      CHECK(state.mode == g1motor::kIdle && !state.fault());
    }
  } catch (const std::exception& e) {
    std::printf("exception: %s\n", e.what());
    ++g1motor_test::g_failures;
  }
  return g1motor_test::Finish("hardware_test");
}

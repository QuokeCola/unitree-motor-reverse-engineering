// Hold every motor found where it is for a few seconds, and report the polling rate.
// usage: g1motor_position_hold [seconds] [port]
// Gains are the ones tuned on a bench wrist (ids 4, 5, 6) at 24 V; start lower on other joints.
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <map>
#include <vector>

#include "g1motor/g1motor.hpp"

int main(int argc, char** argv) {
    const double seconds = argc > 1 ? std::atof(argv[1]) : 3.0;
    try {
        g1motor::MotorBus bus(argc > 2 ? argv[2] : "");
        std::vector<int> ids;
        for (const auto& [id, s] : bus.scan()) ids.push_back(id);
        if (ids.empty()) { std::puts("no motors found"); return 1; }
        const auto start = bus.enable(ids);
        std::map<int, int> worst;
        long cycles = 0;
        const auto t0 = std::chrono::steady_clock::now();
        while (std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() < seconds) {
            for (int id : ids) {
                const int kd = id == 4 ? 40 : 80;
                if (auto s = bus.command(id, start.at(id).pos, 0, 0, 300, kd)) {
                    const int err = std::abs(s->pos - start.at(id).pos);
                    if (err > worst[id]) worst[id] = err;
                }
            }
            ++cycles;
        }
        for (int id : ids) std::printf("id %d: held %d counts, largest error %d counts\n", id, start.at(id).pos, worst[id]);
        std::printf("%.0f exchanges per second per motor\n", cycles / seconds);
        return 0;                                  // the destructor returns the motors to idle
    } catch (const std::exception& e) {
        std::fprintf(stderr, "error: %s\n", e.what());
        return 2;
    }
}

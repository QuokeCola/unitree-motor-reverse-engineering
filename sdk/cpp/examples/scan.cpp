// List the motors on the bus.  usage: g1motor_scan [port]
#include <cstdio>
#include <exception>

#include "g1motor/g1motor.hpp"

int main(int argc, char** argv) {
    try {
        g1motor::MotorBus bus(argc > 1 ? argv[1] : "");
        const auto found = bus.scan();
        if (found.empty()) std::puts("no motors answered on ids 0-14");
        for (const auto& [id, s] : found)
            std::printf("id %d: mode %d  pos %9d  vel %6d  tau %5d  %4.1f V  coil %d C  shell %d C%s\n", id, s.mode, s.pos, s.vel, s.tau,
                        s.voltage(), s.temp_coil, s.temp_shell, s.fault() ? "  FAULT" : "");
        return found.empty() ? 1 : 0;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "error: %s\n", e.what());
        return 2;
    }
}

#include "g1motor/bus.hpp"

#include <chrono>
#include <cstdio>
#include <thread>

namespace g1motor {
namespace {
using Clock = std::chrono::steady_clock;
double since(Clock::time_point t0) { return std::chrono::duration<double>(Clock::now() - t0).count(); }
}  // namespace

MotorBus::MotorBus(const std::string& port, int reply_timeout_us, bool check_silent) : reply_timeout_us_(reply_timeout_us) {
    port_.open(port.empty() ? SerialPort::find_port() : port, kBaud);
    if (check_silent && !listen(0.3).empty()) {
        port_.close();
        throw BusBusyError("bus is not silent: a controller is already polling the motors. "
                           "Two masters on one RS-485 bus corrupt each other; listen only.");
    }
}

MotorBus::~MotorBus() {
    try { close(); } catch (...) {}
}

void MotorBus::close() {
    if (!port_.is_open()) return;
    if (!enabled_.empty()) disable();
    port_.close();
}

std::vector<std::uint8_t> MotorBus::listen(double seconds) {
    std::vector<std::uint8_t> out;
    std::uint8_t buf[4096];
    const auto t0 = Clock::now();
    while (since(t0) < seconds) {
        const std::size_t n = port_.read(buf, sizeof(buf), 1000);
        out.insert(out.end(), buf, buf + n);
    }
    return out;
}

std::optional<MotorState> MotorBus::transfer(const MotorCmd& cmd) {
    const auto frame = cmd.encode();
    port_.flush_input();
    port_.write(frame.data(), frame.size());
    std::uint8_t rx[256];
    std::size_t have = 0;
    const auto t0 = Clock::now();
    while (true) {
        const int left = reply_timeout_us_ - static_cast<int>(since(t0) * 1e6);
        if (left <= 0 || have >= sizeof(rx)) return std::nullopt;
        have += port_.read(rx + have, sizeof(rx) - have, left);
        if (have >= kReplyLen) {
            if (auto s = find_reply(rx, have)) {
                if (s->id != cmd.id) return std::nullopt;
                return s;
            }
        }
    }
}

std::uint8_t MotorBus::byte3(int id, bool flag) {
    // The G1 controller toggles this byte between 0x80 and 0x90 about every 8 frames. A motor enters
    // run mode without it; it is reproduced because every motion test so far was done with it.
    const unsigned n = count_[id]++;
    if (!flag) return 0;
    return (n / 8) % 2 == 0 ? 0x80 : 0x90;
}

std::map<int, MotorState> MotorBus::scan(int tries) {
    std::map<int, MotorState> found;
    for (int id = 0; id < kBroadcastId; ++id) {
        for (int t = 0; t < tries; ++t) {
            MotorCmd c;
            c.id = id;
            if (auto s = transfer(c)) { found[id] = *s; break; }
        }
    }
    return found;
}

std::optional<MotorState> MotorBus::read(int id) {
    if (is_enabled(id)) return command(id);
    MotorCmd c;
    c.id = id;
    return transfer(c);
}

std::map<int, MotorState> MotorBus::enable(const std::vector<int>& ids) {
    std::map<int, MotorState> last;
    const struct { bool flag; int mode; int cycles; } steps[] = {{false, kIdle, 20}, {true, kIdle, 50}, {true, kRun, 5}};
    for (const auto& st : steps) {
        for (int n = 0; n < st.cycles; ++n) {
            for (int id : ids) {
                MotorCmd c;
                c.id = id; c.mode = st.mode; c.flag = st.flag; c.byte3 = byte3(id, st.flag);
                if (auto s = transfer(c)) last[id] = *s;
            }
        }
    }
    auto back_to_idle = [&] {
        for (int n = 0; n < 20; ++n)
            for (int id : ids) { MotorCmd c; c.id = id; transfer(c); }
    };
    for (int id : ids) {
        auto it = last.find(id);
        if (it == last.end()) { back_to_idle(); throw MotorFaultError("motor " + std::to_string(id) + " did not answer"); }
        if (it->second.mode != kRun) {
            back_to_idle();
            char why[96];
            std::snprintf(why, sizeof(why), "status word 0x%08x; call restart() to clear a latched fault", it->second.error);
            throw MotorFaultError("motor " + std::to_string(id) + " did not enter run mode: " + why);
        }
    }
    for (int id : ids) enabled_.insert(id);
    for (int id : ids) {
        auto j = joints_.find(id);
        if (j == joints_.end()) continue;
        try { j->second.sync(last[id].pos); }
        catch (const std::exception& e) { disable(ids); throw MotorFaultError("motor " + std::to_string(id) + ": " + e.what()); }
    }
    return last;
}

void MotorBus::disable(const std::vector<int>& ids_in) {
    std::vector<int> ids = ids_in.empty() ? std::vector<int>(enabled_.begin(), enabled_.end()) : ids_in;
    for (int n = 0; n < 20; ++n)
        for (int id : ids) { MotorCmd c; c.id = id; transfer(c); }
    for (int id : ids) enabled_.erase(id);
}

std::map<int, MotorState> MotorBus::restart(const std::vector<int>& ids, double wait_seconds) {
    for (int id : ids) {
        MotorCmd c;
        c.id = id; c.mode = kRestart;
        transfer(c);
        enabled_.erase(id);
    }
    std::this_thread::sleep_for(std::chrono::duration<double>(wait_seconds));
    std::map<int, MotorState> after;
    for (int id : ids) {
        for (int t = 0; t < 5; ++t) {
            MotorCmd c;
            c.id = id;
            if (auto s = transfer(c)) { after[id] = *s; break; }
        }
    }
    for (auto& [id, s] : after) {
        auto j = joints_.find(id);
        if (j != joints_.end()) {
            try { j->second.sync(s.pos); }
            catch (const std::exception& e) { throw MotorFaultError("motor " + std::to_string(id) + ": " + e.what()); }
        }
    }
    return after;
}

std::optional<MotorState> MotorBus::command(int id, std::int32_t pos, int vel, int tau, int kp, int kd) {
    if (!is_enabled(id)) throw std::runtime_error("motor " + std::to_string(id) + " is not enabled; call enable() first");
    if (kp) {
        auto j = joints_.find(id);
        if (j != joints_.end()) pos = j->second.clamp(pos);
    }
    MotorCmd c;
    c.id = id; c.mode = kRun; c.flag = true; c.byte3 = byte3(id, true);
    c.pos = pos; c.vel = vel; c.tau = tau; c.kp = kp; c.kd = kd;
    return transfer(c);
}

}  // namespace g1motor

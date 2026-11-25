#include "g1motor/motor_bus.h"

#include <chrono>
#include <cstdio>
#include <thread>

namespace g1motor {
namespace {

/// Clock used for timeouts.
using Clock = std::chrono::steady_clock;

/// @return Seconds elapsed since `start`.
double seconds_since(Clock::time_point start) {
  return std::chrono::duration<double>(Clock::now() - start).count();
}

/// @return The idle poll (mode 0, all fields zero) for a motor.
MotorCmd idle_cmd(int id) {
  MotorCmd cmd;
  cmd.id = id;
  return cmd;
}

}  // namespace

MotorBus::MotorBus(const std::string& port, double reply_timeout,
                   bool check_silent)
    : reply_timeout_(reply_timeout) {
  port_.open(port.empty() ? SerialPort::find_port() : port, BAUD);
  if (check_silent && !listen(0.3).empty()) {
    port_.close();
    throw BusBusyError(
        "bus is not silent: a controller is already polling the motors");
  }
}

MotorBus::~MotorBus() {
  try {
    close();
  } catch (...) {
  }
}

void MotorBus::close() {
  if (!port_.is_open()) return;
  if (!enabled_.empty()) disable();
  port_.close();
}

std::vector<std::uint8_t> MotorBus::listen(double seconds) {
  std::vector<std::uint8_t> bytes;
  std::uint8_t buffer[4096];
  const auto start = Clock::now();
  while (seconds_since(start) < seconds) {
    const std::size_t n = port_.read(buffer, sizeof(buffer), 1000);
    bytes.insert(bytes.end(), buffer, buffer + n);
  }
  return bytes;
}

std::optional<MotorState> MotorBus::transfer(const MotorCmd& cmd) {
  const auto frame = cmd.encode();
  port_.flush_input();
  port_.write(frame.data(), frame.size());
  std::uint8_t rx[256];
  std::size_t have = 0;
  const auto start = Clock::now();
  while (true) {
    const int left =
        static_cast<int>((reply_timeout_ - seconds_since(start)) * 1e6);
    if (left <= 0 || have >= sizeof(rx)) return std::nullopt;
    have += port_.read(rx + have, sizeof(rx) - have, left);
    if (have < REPLY_LEN) continue;
    if (auto state = find_reply(rx, have)) {
      if (state->id != cmd.id) return std::nullopt;
      return state;
    }
  }
}

std::uint8_t MotorBus::next_byte3(int id, bool flag) {
  const unsigned n = frame_count_[id]++;
  if (!flag) return 0;
  return (n / 8) % 2 == 0 ? 0x80 : 0x90;
}

void MotorBus::send_idle(const std::vector<int>& ids, int cycles) {
  for (int n = 0; n < cycles; ++n) {
    for (int id : ids) transfer(idle_cmd(id));
  }
}

std::map<int, MotorState> MotorBus::scan(int tries) {
  std::map<int, MotorState> found;
  for (int id = 0; id < BROADCAST_ID; ++id) {
    for (int attempt = 0; attempt < tries; ++attempt) {
      if (auto state = transfer(idle_cmd(id))) {
        found[id] = *state;
        break;
      }
    }
  }
  return found;
}

std::optional<MotorState> MotorBus::read(int id) {
  if (is_enabled(id)) return command(id);
  return transfer(idle_cmd(id));
}

std::map<int, MotorState> MotorBus::enable(const std::vector<int>& ids) {
  // Same order as the G1 controller at boot: idle, flag bit, then run mode.
  struct Step {
    bool flag;
    int mode;
    int cycles;
  };
  const Step steps[] = {
      {false, MODE_IDLE, 20}, {true, MODE_IDLE, 50}, {true, MODE_RUN, 5}};
  std::map<int, MotorState> last;
  for (const Step& step : steps) {
    for (int n = 0; n < step.cycles; ++n) {
      for (int id : ids) {
        MotorCmd cmd;
        cmd.id = id;
        cmd.mode = step.mode;
        cmd.flag = step.flag;
        cmd.byte3 = next_byte3(id, step.flag);
        if (auto state = transfer(cmd)) last[id] = *state;
      }
    }
  }
  for (int id : ids) {
    const auto it = last.find(id);
    if (it == last.end()) {
      send_idle(ids, 20);
      throw MotorFaultError("motor " + std::to_string(id) + " did not answer");
    }
    if (it->second.mode != MODE_RUN) {
      send_idle(ids, 20);
      char status[16];
      std::snprintf(status, sizeof(status), "0x%08x", it->second.error);
      throw MotorFaultError("motor " + std::to_string(id) +
                            " did not enter run mode (status " + status +
                            "); restart() clears a latched fault");
    }
  }
  for (int id : ids) enabled_.insert(id);
  for (int id : ids) {
    const auto it = joints_.find(id);
    if (it == joints_.end()) continue;
    try {
      it->second.sync(last[id].pos);
    } catch (const std::exception& e) {
      disable(ids);
      throw MotorFaultError("motor " + std::to_string(id) + ": " + e.what());
    }
  }
  return last;
}

void MotorBus::disable(const std::vector<int>& ids) {
  const std::vector<int> targets =
      ids.empty() ? std::vector<int>(enabled_.begin(), enabled_.end()) : ids;
  send_idle(targets, 20);
  for (int id : targets) enabled_.erase(id);
}

std::map<int, MotorState> MotorBus::restart(const std::vector<int>& ids,
                                            double wait) {
  for (int id : ids) {
    MotorCmd cmd;
    cmd.id = id;
    cmd.mode = MODE_RESTART;
    transfer(cmd);
    enabled_.erase(id);
  }
  std::this_thread::sleep_for(std::chrono::duration<double>(wait));
  std::map<int, MotorState> after;
  for (int id : ids) {
    for (int attempt = 0; attempt < 5; ++attempt) {
      if (auto state = transfer(idle_cmd(id))) {
        after[id] = *state;
        break;
      }
    }
  }
  for (const auto& [id, state] : after) {
    const auto it = joints_.find(id);
    if (it == joints_.end()) continue;
    try {
      it->second.sync(state.pos);
    } catch (const std::exception& e) {
      throw MotorFaultError("motor " + std::to_string(id) + ": " + e.what());
    }
  }
  return after;
}

std::optional<MotorState> MotorBus::command(int id, std::int32_t pos, int vel,
                                            int tau, int kp, int kd) {
  if (!is_enabled(id)) {
    throw std::runtime_error("motor " + std::to_string(id) +
                             " is not enabled; call enable() first");
  }
  if (kp != 0) {
    const auto it = joints_.find(id);
    if (it != joints_.end()) pos = it->second.clamp(pos);
  }
  MotorCmd cmd;
  cmd.id = id;
  cmd.mode = MODE_RUN;
  cmd.flag = true;
  cmd.byte3 = next_byte3(id, true);
  cmd.pos = pos;
  cmd.vel = vel;
  cmd.tau = tau;
  cmd.kp = kp;
  cmd.kd = kd;
  return transfer(cmd);
}

void MotorBus::set_joint(int id, const Joint& joint) { joints_[id] = joint; }

Joint& MotorBus::joint(int id) { return joints_.at(id); }

bool MotorBus::is_enabled(int id) const { return enabled_.count(id) != 0; }

}  // namespace g1motor

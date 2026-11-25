#include "g1motor/motor_bus.h"

#include <chrono>
#include <cstdio>
#include <thread>

namespace g1motor {
namespace {

/// Clock used for timeouts.
using Clock = std::chrono::steady_clock;

/// @return Seconds elapsed since `start`.
double SecondsSince(Clock::time_point start) {
  return std::chrono::duration<double>(Clock::now() - start).count();
}

/// @return The idle poll (mode 0, all fields zero) for a motor.
MotorCmd IdleCmd(int id) {
  MotorCmd cmd;
  cmd.id = id;
  return cmd;
}

}  // namespace

MotorBus::MotorBus(const std::string& port, int reply_timeout_us,
                   bool check_silent)
    : reply_timeout_us_(reply_timeout_us) {
  port_.Open(port.empty() ? SerialPort::FindPort() : port, kBaud);
  if (check_silent && !Listen(0.3).empty()) {
    port_.Close();
    throw BusBusyError(
        "bus is not silent: a controller is already polling the motors");
  }
}

MotorBus::~MotorBus() {
  try {
    Close();
  } catch (...) {
  }
}

void MotorBus::Close() {
  if (!port_.is_open()) return;
  if (!enabled_.empty()) Disable();
  port_.Close();
}

std::vector<std::uint8_t> MotorBus::Listen(double seconds) {
  std::vector<std::uint8_t> bytes;
  std::uint8_t buffer[4096];
  const auto start = Clock::now();
  while (SecondsSince(start) < seconds) {
    const std::size_t n = port_.Read(buffer, sizeof(buffer), 1000);
    bytes.insert(bytes.end(), buffer, buffer + n);
  }
  return bytes;
}

std::optional<MotorState> MotorBus::Transfer(const MotorCmd& cmd) {
  const auto frame = cmd.Encode();
  port_.FlushInput();
  port_.Write(frame.data(), frame.size());
  std::uint8_t rx[256];
  std::size_t have = 0;
  const auto start = Clock::now();
  while (true) {
    const int left =
        reply_timeout_us_ - static_cast<int>(SecondsSince(start) * 1e6);
    if (left <= 0 || have >= sizeof(rx)) return std::nullopt;
    have += port_.Read(rx + have, sizeof(rx) - have, left);
    if (have < kReplyLen) continue;
    if (auto state = FindReply(rx, have)) {
      if (state->id != cmd.id) return std::nullopt;
      return state;
    }
  }
}

std::uint8_t MotorBus::NextByte3(int id, bool flag) {
  const unsigned n = frame_count_[id]++;
  if (!flag) return 0;
  return (n / 8) % 2 == 0 ? 0x80 : 0x90;
}

void MotorBus::SendIdle(const std::vector<int>& ids, int cycles) {
  for (int n = 0; n < cycles; ++n) {
    for (int id : ids) Transfer(IdleCmd(id));
  }
}

std::map<int, MotorState> MotorBus::Scan(int tries) {
  std::map<int, MotorState> found;
  for (int id = 0; id < kBroadcastId; ++id) {
    for (int attempt = 0; attempt < tries; ++attempt) {
      if (auto state = Transfer(IdleCmd(id))) {
        found[id] = *state;
        break;
      }
    }
  }
  return found;
}

std::optional<MotorState> MotorBus::Read(int id) {
  if (is_enabled(id)) return Command(id);
  return Transfer(IdleCmd(id));
}

std::map<int, MotorState> MotorBus::Enable(const std::vector<int>& ids) {
  // Same order as the G1 controller at boot: idle, flag bit, then run mode.
  struct Step {
    bool flag;
    int mode;
    int cycles;
  };
  const Step steps[] = {{false, kIdle, 20}, {true, kIdle, 50}, {true, kRun, 5}};
  std::map<int, MotorState> last;
  for (const Step& step : steps) {
    for (int n = 0; n < step.cycles; ++n) {
      for (int id : ids) {
        MotorCmd cmd;
        cmd.id = id;
        cmd.mode = step.mode;
        cmd.flag = step.flag;
        cmd.byte3 = NextByte3(id, step.flag);
        if (auto state = Transfer(cmd)) last[id] = *state;
      }
    }
  }
  for (int id : ids) {
    const auto it = last.find(id);
    if (it == last.end()) {
      SendIdle(ids, 20);
      throw MotorFaultError("motor " + std::to_string(id) + " did not answer");
    }
    if (it->second.mode != kRun) {
      SendIdle(ids, 20);
      char status[16];
      std::snprintf(status, sizeof(status), "0x%08x", it->second.error);
      throw MotorFaultError("motor " + std::to_string(id) +
                            " did not enter run mode (status " + status +
                            "); Restart() clears a latched fault");
    }
  }
  for (int id : ids) enabled_.insert(id);
  for (int id : ids) {
    const auto it = joints_.find(id);
    if (it == joints_.end()) continue;
    try {
      it->second.Sync(last[id].pos);
    } catch (const std::exception& e) {
      Disable(ids);
      throw MotorFaultError("motor " + std::to_string(id) + ": " + e.what());
    }
  }
  return last;
}

void MotorBus::Disable(const std::vector<int>& ids) {
  const std::vector<int> targets =
      ids.empty() ? std::vector<int>(enabled_.begin(), enabled_.end()) : ids;
  SendIdle(targets, 20);
  for (int id : targets) enabled_.erase(id);
}

std::map<int, MotorState> MotorBus::Restart(const std::vector<int>& ids,
                                            double wait_seconds) {
  for (int id : ids) {
    MotorCmd cmd;
    cmd.id = id;
    cmd.mode = kRestart;
    Transfer(cmd);
    enabled_.erase(id);
  }
  std::this_thread::sleep_for(std::chrono::duration<double>(wait_seconds));
  std::map<int, MotorState> after;
  for (int id : ids) {
    for (int attempt = 0; attempt < 5; ++attempt) {
      if (auto state = Transfer(IdleCmd(id))) {
        after[id] = *state;
        break;
      }
    }
  }
  for (const auto& [id, state] : after) {
    const auto it = joints_.find(id);
    if (it == joints_.end()) continue;
    try {
      it->second.Sync(state.pos);
    } catch (const std::exception& e) {
      throw MotorFaultError("motor " + std::to_string(id) + ": " + e.what());
    }
  }
  return after;
}

std::optional<MotorState> MotorBus::Command(int id, std::int32_t pos, int vel,
                                            int tau, int kp, int kd) {
  if (!is_enabled(id)) {
    throw std::runtime_error("motor " + std::to_string(id) +
                             " is not enabled; call Enable() first");
  }
  if (kp != 0) {
    const auto it = joints_.find(id);
    if (it != joints_.end()) pos = it->second.Clamp(pos);
  }
  MotorCmd cmd;
  cmd.id = id;
  cmd.mode = kRun;
  cmd.flag = true;
  cmd.byte3 = NextByte3(id, true);
  cmd.pos = pos;
  cmd.vel = vel;
  cmd.tau = tau;
  cmd.kp = kp;
  cmd.kd = kd;
  return Transfer(cmd);
}

}  // namespace g1motor

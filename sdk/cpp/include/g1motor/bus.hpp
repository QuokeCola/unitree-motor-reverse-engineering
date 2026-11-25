// MotorBus: talk to G1 joint motors on an RS-485 chain that has no controller on it.
//
//   g1motor::MotorBus bus;                         // the only FTDI adapter, 6 Mbaud
//   for (auto& [id, s] : bus.scan()) ...           // who is there
//   auto start = bus.enable({4, 5, 6});            // run mode, zero torque: joints are free
//   bus.command(4, start[4].pos, 0, 0, 300, 40);   // hold joint 4 where it is; call in a steady loop
//   // the destructor returns the motors to idle, which brakes them
#pragma once

#include <cstdint>
#include <map>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include "g1motor/joint.hpp"
#include "g1motor/protocol.hpp"
#include "g1motor/serial_port.hpp"

namespace g1motor {

struct BusBusyError : std::runtime_error { using std::runtime_error::runtime_error; };     // a controller is already polling
struct MotorFaultError : std::runtime_error { using std::runtime_error::runtime_error; };  // a motor would not enter run mode

class MotorBus {
public:
    // port empty = the only FTDI adapter (Linux). Throws BusBusyError if the bus is not silent.
    explicit MotorBus(const std::string& port = "", int reply_timeout_us = 4000, bool check_silent = true);
    ~MotorBus();
    MotorBus(const MotorBus&) = delete;
    MotorBus& operator=(const MotorBus&) = delete;

    void close();

    // Receive without transmitting, for this many seconds.
    std::vector<std::uint8_t> listen(double seconds);
    // Send one frame and wait for that motor's reply. Empty on timeout or bad checksum.
    std::optional<MotorState> transfer(const MotorCmd& cmd);

    // Poll ids 0-14 with the idle frame and return who answers.
    std::map<int, MotorState> scan(int tries = 3);
    // Fresh state. On an enabled motor this sends zero torque, which drops a position hold.
    std::optional<MotorState> read(int id);

    // Run mode with zero torque. Joints attached with set_joint() are re-referenced.
    std::map<int, MotorState> enable(const std::vector<int>& ids);
    // Back to idle, which brakes the joint passively. This is also how to stop a moving joint.
    void disable(const std::vector<int>& ids = {});
    // Mode 7: clears a latched fault; about 0.3 s. Positions may come back shifted by whole output turns.
    std::map<int, MotorState> restart(const std::vector<int>& ids, double wait_seconds = 0.5);

    // One hybrid command in raw counts to an enabled motor. Keep calling it in a steady loop:
    // a motor that is no longer polled drops to idle after 0.5-1 s.
    std::optional<MotorState> command(int id, std::int32_t pos = 0, int vel = 0, int tau = 0, int kp = 0, int kd = 0);

    void set_joint(int id, const Joint& joint) { joints_[id] = joint; }
    Joint& joint(int id) { return joints_.at(id); }
    bool is_enabled(int id) const { return enabled_.count(id) != 0; }

private:
    std::uint8_t byte3(int id, bool flag);

    SerialPort port_;
    int reply_timeout_us_;
    std::set<int> enabled_;
    std::map<int, Joint> joints_;
    std::map<int, unsigned> count_;
};

}  // namespace g1motor

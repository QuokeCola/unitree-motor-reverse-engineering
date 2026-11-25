/// @file motor_bus.h
/// @brief MotorBus: drive G1 joint motors on a bus without a controller.

#ifndef G1MOTOR_MOTOR_BUS_H_
#define G1MOTOR_MOTOR_BUS_H_

#include <cstdint>
#include <map>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include "g1motor/joint.h"
#include "g1motor/protocol.h"
#include "g1motor/serial_port.h"

namespace g1motor {

/// @brief Thrown when a controller is already polling the bus.
struct BusBusyError : std::runtime_error {
  using std::runtime_error::runtime_error;
};

/// @brief Thrown when a motor does not answer or will not enter run mode.
struct MotorFaultError : std::runtime_error {
  using std::runtime_error::runtime_error;
};

/// @brief One RS-485 chain of motors, with this program as the only master.
///
/// @code
///   g1motor::MotorBus bus;
///   auto start = bus.Enable({4, 5, 6});   // run mode, joints free
///   bus.Command(4, start[4].pos, 0, 0, 300, 40);   // call in a steady loop
/// @endcode
///
/// Motors answer only when polled, so every command returns that motor's
/// fresh state. A motor that is no longer polled drops to idle after
/// 0.5-1 s. The destructor returns enabled motors to idle.
class MotorBus {
 public:
  /// @brief Opens the adapter at 6 Mbaud.
  /// @param port Device name; empty selects the only FTDI adapter (Linux).
  /// @param reply_timeout_us Longest wait for a reply, in microseconds.
  /// @param check_silent Listen first and refuse a bus that has traffic.
  /// @throws BusBusyError if check_silent is set and the bus is not silent.
  /// @throws std::runtime_error if the port cannot be opened.
  explicit MotorBus(const std::string& port = "", int reply_timeout_us = 4000,
                    bool check_silent = true);
  ~MotorBus();
  MotorBus(const MotorBus&) = delete;
  MotorBus& operator=(const MotorBus&) = delete;

  /// @brief Returns enabled motors to idle and closes the port.
  void Close();

  /// @brief Receives without transmitting.
  /// @param seconds How long to listen.
  /// @return The raw bytes received.
  std::vector<std::uint8_t> Listen(double seconds);

  /// @brief Sends one frame and waits for that motor's reply.
  /// @return The reply, or empty on timeout or a bad checksum.
  std::optional<MotorState> Transfer(const MotorCmd& cmd);

  /// @brief Polls ids 0-14 with the idle frame.
  /// @param tries Attempts per id.
  /// @return The motors that answered, by id.
  std::map<int, MotorState> Scan(int tries = 3);

  /// @brief Reads a motor's state.
  ///
  /// On an enabled motor this sends a zero-torque command, which drops a
  /// position hold.
  std::optional<MotorState> Read(int id);

  /// @brief Puts motors in run mode with zero torque; the joints are free.
  ///
  /// Joints attached with set_joint() are re-referenced to the reading.
  /// @return The state of each motor.
  /// @throws MotorFaultError if a motor does not reach run mode.
  std::map<int, MotorState> Enable(const std::vector<int>& ids);

  /// @brief Returns motors to idle, which brakes the joint passively.
  ///
  /// This is also the way to stop a moving joint.
  /// @param ids Motors to idle; empty means every enabled motor.
  void Disable(const std::vector<int>& ids = {});

  /// @brief Restarts motors (mode 7), clearing a latched fault.
  ///
  /// The position reading may come back shifted by whole output turns;
  /// attached joints are re-referenced.
  /// @param ids Motors to restart.
  /// @param wait_seconds Time allowed for the restart (about 0.3 s needed).
  /// @return The state of each motor after the restart.
  std::map<int, MotorState> Restart(const std::vector<int>& ids,
                                    double wait_seconds = 0.5);

  /// @brief Sends one hybrid command, in raw counts, to an enabled motor.
  ///
  /// Position targets are limited to the end stops of an attached joint.
  /// @return The motor's state, or empty if it did not reply.
  /// @throws std::runtime_error if the motor is not enabled.
  std::optional<MotorState> Command(int id, std::int32_t pos = 0, int vel = 0,
                                    int tau = 0, int kp = 0, int kd = 0);

  /// @brief Attaches a joint description to a motor.
  void set_joint(int id, const Joint& joint) { joints_[id] = joint; }
  /// @return The joint attached to a motor.
  /// @throws std::out_of_range if none is attached.
  Joint& joint(int id) { return joints_.at(id); }
  /// @return True if the motor was enabled through this object.
  bool is_enabled(int id) const { return enabled_.count(id) != 0; }

 private:
  /// Sequence byte as the G1 controller sends it: 0x80 / 0x90 every 8 frames.
  std::uint8_t NextByte3(int id, bool flag);
  /// Sends idle polls to the given motors.
  void SendIdle(const std::vector<int>& ids, int cycles);

  SerialPort port_;                      ///< The open serial port.
  int reply_timeout_us_;                 ///< Longest wait for a reply.
  std::set<int> enabled_;                ///< Motors enabled here.
  std::map<int, Joint> joints_;          ///< Joints attached, by id.
  std::map<int, unsigned> frame_count_;  ///< Frames sent per motor.
};

}  // namespace g1motor

#endif  // G1MOTOR_MOTOR_BUS_H_

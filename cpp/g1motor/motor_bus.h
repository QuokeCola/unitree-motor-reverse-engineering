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

/// @brief Raised when a controller is already polling the bus.
struct BusBusyError : std::runtime_error {
  using std::runtime_error::runtime_error;
};

/// @brief Raised when a motor does not answer or will not enter run mode.
struct MotorFaultError : std::runtime_error {
  using std::runtime_error::runtime_error;
};

/// @brief One RS-485 chain of motors, with this program as the only master.
///
/// @code
///   g1motor::MotorBus bus;
///   auto start = bus.enable({4, 5, 6});   // run mode, joints free
///   bus.command(4, start[4].pos, 0, 0, 300, 40);   // in a steady loop
/// @endcode
///
/// Motors answer only when polled, so every command returns that motor's
/// fresh state. A motor that is no longer polled drops to idle after
/// 0.5-1 s. The destructor returns enabled motors to idle.
class MotorBus {
 public:
  /// @brief Opens the adapter at 6 Mbaud.
  /// @param port Device name; empty selects the only FTDI adapter.
  /// @param reply_timeout Longest wait for a reply, in seconds.
  /// @param check_silent Listen first and refuse a bus that has traffic.
  /// @throws BusBusyError if the bus is not silent.
  /// @throws std::runtime_error if the port cannot be opened.
  explicit MotorBus(const std::string& port = "", double reply_timeout = 0.004,
                    bool check_silent = true);
  ~MotorBus();
  MotorBus(const MotorBus&) = delete;
  MotorBus& operator=(const MotorBus&) = delete;

  /// @brief Returns enabled motors to idle and closes the port.
  void close();

  /// @brief Receives without transmitting.
  /// @param seconds How long to listen.
  /// @return The raw bytes received.
  std::vector<std::uint8_t> listen(double seconds);

  /// @brief Sends one frame and waits for that motor's reply.
  /// @param cmd Command to send.
  /// @return The reply, or empty on timeout or a bad checksum.
  std::optional<MotorState> transfer(const MotorCmd& cmd);

  /// @brief Polls ids 0-14 with the idle frame.
  /// @param tries Attempts per id.
  /// @return The motors that answered, by id.
  std::map<int, MotorState> scan(int tries = 3);

  /// @brief Reads a motor's state.
  ///
  /// On an enabled motor this sends a zero-torque command, which drops a
  /// position hold.
  /// @param id Motor to read.
  /// @return The state, or empty if it did not reply.
  std::optional<MotorState> read(int id);

  /// @brief Puts motors in run mode with zero torque; the joints are free.
  ///
  /// Joints attached with set_joint() are re-referenced to the reading.
  /// @param ids Motors to enable.
  /// @return The state of each motor.
  /// @throws MotorFaultError if a motor does not reach run mode.
  std::map<int, MotorState> enable(const std::vector<int>& ids);

  /// @brief Returns motors to idle, which brakes the joint passively.
  ///
  /// This is also the way to stop a moving joint.
  /// @param ids Motors to idle; empty means every enabled motor.
  void disable(const std::vector<int>& ids = {});

  /// @brief Restarts motors (mode 7), clearing a latched fault.
  ///
  /// The position reading may come back shifted by whole output turns;
  /// attached joints are re-referenced.
  /// @param ids Motors to restart.
  /// @param wait Seconds allowed for the restart (about 0.3 s needed).
  /// @return The state of each motor after the restart.
  /// @throws MotorFaultError if an attached joint no longer fits.
  std::map<int, MotorState> restart(const std::vector<int>& ids,
                                    double wait = 0.5);

  /// @brief Sends one hybrid command, in raw counts, to an enabled motor.
  ///
  /// Position targets are limited to the end stops of an attached joint.
  /// @param id Motor to command.
  /// @param pos Position target.
  /// @param vel Velocity target.
  /// @param tau Feed-forward torque.
  /// @param kp Position gain.
  /// @param kd Damping gain.
  /// @return The motor's state, or empty if it did not reply.
  /// @throws std::runtime_error if the motor is not enabled.
  std::optional<MotorState> command(int id, std::int32_t pos = 0, int vel = 0,
                                    int tau = 0, int kp = 0, int kd = 0);

  /// @brief Attaches a joint description to a motor.
  /// @param id Motor the joint belongs to.
  /// @param joint Gear ratio, end stops and zero.
  void set_joint(int id, const Joint& joint);

  /// @brief The joint attached to a motor.
  /// @param id Motor id.
  /// @return The joint, as re-referenced by enable() and restart().
  /// @throws std::out_of_range if none is attached.
  Joint& joint(int id);

  /// @brief Whether a motor was enabled through this object.
  /// @param id Motor id.
  /// @return True if it is enabled.
  bool is_enabled(int id) const;

 private:
  /// Sequence byte as the G1 controller sends it: 0x80 / 0x90 every 8 frames.
  std::uint8_t next_byte3(int id, bool flag);
  /// Sends idle polls to the given motors.
  void send_idle(const std::vector<int>& ids, int cycles);

  SerialPort port_;                      ///< The open serial port.
  double reply_timeout_;                 ///< Longest wait for a reply, s.
  std::set<int> enabled_;                ///< Motors enabled here.
  std::map<int, Joint> joints_;          ///< Joints attached, by id.
  std::map<int, unsigned> frame_count_;  ///< Frames sent per motor.
};

}  // namespace g1motor

#endif  // G1MOTOR_MOTOR_BUS_H_

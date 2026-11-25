/// @file protocol.h
/// @brief Wire format of the Unitree G1 joint motor bus: frames and checksum.
///
/// Unofficial and reverse engineered. All values are raw counts as on the wire.

#ifndef G1MOTOR_PROTOCOL_H_
#define G1MOTOR_PROTOCOL_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace g1motor {

/// Bus bit rate in baud (8N1).
inline constexpr int kBaud = 6000000;
/// Length of a command frame in bytes.
inline constexpr std::size_t kCmdLen = 20;
/// Length of a reply frame in bytes.
inline constexpr std::size_t kReplyLen = 26;
/// Id reserved as broadcast in the public GO-M8010-6 protocol; never polled.
inline constexpr int kBroadcastId = 15;

/// @brief Mode values carried in bits 4-6 of the id/mode byte.
///
/// Modes 3 and 5 are accepted and brake the joint; mode 4 is accepted and
/// leaves it free; all three ignore the command fields. Mode 2 is ignored.
/// Mode 6 sets the position zero with a calibration payload and is not sent
/// by this library.
enum Mode : int {
  kIdle = 0,     ///< Commands ignored; joint passively braked.
  kRun = 1,      ///< Hybrid torque / position / velocity command.
  kRestart = 7,  ///< Motor restarts (about 0.3 s); clears a latched fault.
};

/// @brief Checksum of the bus: CRC-32 over little-endian 32-bit words.
///
/// Polynomial 0x04C11DB7, initial value 0xFFFFFFFF, no reflection, no final
/// XOR, each word fed most-significant bit first.
/// @param data Bytes to check.
/// @param len Number of bytes; must be a multiple of 4.
/// @return The checksum.
/// @throws std::invalid_argument if len is not a multiple of 4.
std::uint32_t Crc32Core(const std::uint8_t* data, std::size_t len);

/// @brief One command frame (controller to motor).
///
/// The motor applies
/// `tau + kp * (pos - measured) + kd * (vel - measured)`.
struct MotorCmd {
  int id = 0;              ///< Motor id, 0-14.
  int mode = kIdle;        ///< See Mode.
  int tau = 0;             ///< Feed-forward torque, int16.
  int vel = 0;             ///< Velocity target, int16.
  std::int32_t pos = 0;    ///< Position target, int32.
  int kp = 0;              ///< Position gain, int16.
  int kd = 0;              ///< Damping gain, int16.
  bool flag = false;       ///< Top bit of the id/mode byte; not required.
  std::uint8_t byte3 = 0;  ///< Sequence byte; not required.

  /// @brief Builds the 20-byte frame, checksum included.
  /// @throws std::out_of_range if the id or a field does not fit.
  std::array<std::uint8_t, kCmdLen> Encode() const;

  /// @brief Parses a command frame.
  /// @return The command, or empty if the header or checksum is wrong.
  static std::optional<MotorCmd> Decode(const std::uint8_t* frame,
                                        std::size_t len);
};

/// @brief One reply frame (motor to controller).
struct MotorState {
  int id = 0;               ///< Motor id.
  int mode = 0;             ///< Mode the motor is actually in.
  int tau = 0;              ///< Net applied torque, int16.
  int vel = 0;              ///< Velocity; 1 count = 128 position counts/s.
  std::int32_t pos = 0;     ///< Position; 32768 counts per rotor turn.
  int temp_coil = 0;        ///< Winding temperature, deg C.
  int temp_shell = 0;       ///< Shell temperature, deg C.
  int voltage_raw = 0;      ///< Bus voltage, 0.5 V per count.
  std::uint32_t error = 0;  ///< Status word; 0 in normal operation.
  int word18 = 0;           ///< Unknown.
  int byte20 = 0;           ///< Unknown.
  int byte21 = 0;           ///< Unknown.

  /// @return Bus voltage in volts.
  double voltage() const { return voltage_raw * 0.5; }
  /// @return True if the status word is non-zero.
  bool fault() const { return error != 0; }

  /// @brief Parses a reply frame.
  /// @return The state, or empty if the header or checksum is wrong.
  static std::optional<MotorState> Decode(const std::uint8_t* frame,
                                          std::size_t len);
};

/// @brief Finds the first valid reply in a receive buffer.
///
/// Skips echoed command bytes and noise.
/// @return The reply, or empty if none is complete and valid.
std::optional<MotorState> FindReply(const std::uint8_t* buffer,
                                    std::size_t len);

}  // namespace g1motor

#endif  // G1MOTOR_PROTOCOL_H_

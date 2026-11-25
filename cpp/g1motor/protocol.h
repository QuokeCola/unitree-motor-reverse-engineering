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
inline constexpr int BAUD = 6000000;
/// Length of a command frame in bytes.
inline constexpr std::size_t CMD_LEN = 20;
/// Length of a reply frame in bytes.
inline constexpr std::size_t REPLY_LEN = 26;
/// Id reserved as broadcast in the public GO-M8010-6 protocol; never polled.
inline constexpr int BROADCAST_ID = 15;

/// Mode 0: commands ignored; joint passively braked.
inline constexpr int MODE_IDLE = 0;
/// Mode 1: hybrid torque / position / velocity command.
inline constexpr int MODE_RUN = 1;
/// Mode 7: motor restarts (about 0.3 s); clears a latched fault.
inline constexpr int MODE_RESTART = 7;

/// @brief Checksum of the bus: CRC-32 over little-endian 32-bit words.
///
/// Polynomial 0x04C11DB7, initial value 0xFFFFFFFF, no reflection, no final
/// XOR, each word fed most-significant bit first.
/// @param data Bytes to check.
/// @param len Number of bytes; must be a multiple of 4.
/// @return The checksum.
/// @throws std::invalid_argument if len is not a multiple of 4.
std::uint32_t crc32_core(const std::uint8_t* data, std::size_t len);

/// @brief One command frame (controller to motor).
///
/// The motor applies
/// `tau + kp * (pos - measured) + kd * (vel - measured)`.
struct MotorCmd {
  int id = 0;              ///< Motor id, 0-14.
  int mode = MODE_IDLE;    ///< MODE_IDLE, MODE_RUN or MODE_RESTART.
  int tau = 0;             ///< Feed-forward torque, int16.
  int vel = 0;             ///< Velocity target, int16.
  std::int32_t pos = 0;    ///< Position target, int32.
  int kp = 0;              ///< Position gain, int16.
  int kd = 0;              ///< Damping gain, int16.
  bool flag = false;       ///< Top bit of the id/mode byte; not required.
  std::uint8_t byte3 = 0;  ///< Sequence byte; not required.

  /// @brief Builds the 20-byte frame, checksum included.
  /// @return The frame.
  /// @throws std::out_of_range if the id or a field does not fit.
  std::array<std::uint8_t, CMD_LEN> encode() const;

  /// @brief Parses a command frame.
  /// @param frame Exactly 20 bytes.
  /// @param len Number of bytes in frame.
  /// @return The command, or empty if the header or checksum is wrong.
  static std::optional<MotorCmd> decode(const std::uint8_t* frame,
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

  /// @brief Bus voltage.
  /// @return Volts.
  double voltage() const { return voltage_raw * 0.5; }

  /// @brief Whether the motor reports a fault.
  /// @return True if the status word is non-zero.
  bool fault() const { return error != 0; }

  /// @brief Parses a reply frame.
  /// @param frame Exactly 26 bytes.
  /// @param len Number of bytes in frame.
  /// @return The state, or empty if the header or checksum is wrong.
  static std::optional<MotorState> decode(const std::uint8_t* frame,
                                          std::size_t len);
};

/// @brief Finds the first valid reply in a receive buffer.
///
/// Skips echoed command bytes and noise.
/// @param buffer Bytes received so far.
/// @param len Number of bytes in buffer.
/// @return The reply, or empty if none is complete and valid.
std::optional<MotorState> find_reply(const std::uint8_t* buffer,
                                     std::size_t len);

}  // namespace g1motor

#endif  // G1MOTOR_PROTOCOL_H_

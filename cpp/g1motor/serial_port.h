/// @file serial_port.h
/// @brief Serial port at 6 Mbaud using each operating system's own interface.
///
/// Linux: termios2 with BOTHER. macOS: termios and IOSSIOSPEED. Windows:
/// Win32 DCB. Only the Linux path has been run on hardware.

#ifndef G1MOTOR_SERIAL_PORT_H_
#define G1MOTOR_SERIAL_PORT_H_

#include <cstddef>
#include <cstdint>
#include <string>

namespace g1motor {

/// @brief Raw 8N1 serial port with a non-standard bit rate.
class SerialPort {
 public:
  SerialPort() = default;
  ~SerialPort();
  SerialPort(const SerialPort&) = delete;
  SerialPort& operator=(const SerialPort&) = delete;

  /// @brief Opens the port in raw 8N1 mode.
  /// @param port Device name, e.g. /dev/ttyUSB0 or COM5.
  /// @param baud Bit rate.
  /// @throws std::runtime_error if it cannot be opened or refuses the rate.
  void open(const std::string& port, int baud);

  /// @brief Closes the port; does nothing if it is not open.
  void close();

  /// @return True if the port is open.
  bool is_open() const;

  /// @brief Writes all bytes.
  /// @throws std::runtime_error on a write error.
  void write(const std::uint8_t* data, std::size_t len);

  /// @brief Reads what is available, waiting up to a timeout.
  /// @param data Destination buffer.
  /// @param max_len Size of the buffer.
  /// @param timeout_us Longest wait in microseconds.
  /// @return Bytes read; 0 on timeout.
  std::size_t read(std::uint8_t* data, std::size_t max_len, int timeout_us);

  /// @brief Discards bytes received but not yet read.
  void flush_input();

  /// @brief Finds the only FTDI adapter (Linux).
  /// @throws std::runtime_error if none or several are found, or on other
  ///     systems, where the port name must be given.
  static std::string find_port();

 private:
#ifdef _WIN32
  void* handle_ = nullptr;  ///< Win32 handle of the port.
#else
  int fd_ = -1;  ///< File descriptor of the port.
#endif
};

}  // namespace g1motor

#endif  // G1MOTOR_SERIAL_PORT_H_

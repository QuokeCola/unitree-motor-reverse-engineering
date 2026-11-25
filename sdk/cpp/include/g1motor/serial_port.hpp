// Serial port at 6 Mbaud, 8N1, using each operating system's own interface:
//   Linux    termios2 with BOTHER (TCSETS2), FTDI low-latency flag
//   macOS    termios + IOSSIOSPEED
//   Windows  Win32 CreateFile / DCB
// Only the Linux path has been run on hardware.
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace g1motor {

class SerialPort {
public:
    SerialPort() = default;
    ~SerialPort();
    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    // Throws std::runtime_error if the port cannot be opened or the rate is not accepted.
    void open(const std::string& port, int baud);
    void close();
    bool is_open() const;

    void write(const std::uint8_t* data, std::size_t len);
    // Waits up to timeout_us for data; returns the number of bytes read (0 on timeout).
    std::size_t read(std::uint8_t* data, std::size_t max_len, int timeout_us);
    void flush_input();

    // Linux: the only FTDI adapter under /dev/serial/by-id. Other systems: throws, give the port name.
    static std::string find_port();

private:
#ifdef _WIN32
    void* handle_ = nullptr;
#else
    int fd_ = -1;
#endif
};

}  // namespace g1motor

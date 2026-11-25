#include "g1motor/serial_port.h"

#include <stdexcept>
#include <string>
#include <vector>

#if defined(_WIN32)
// ---------------------------------------------------------------- Windows (not
// run on hardware)
#define WIN32_LEAN_AND_MEAN
#include <windows.h>

namespace g1motor {

SerialPort::~SerialPort() { Close(); }
bool SerialPort::is_open() const { return handle_ != nullptr; }

void SerialPort::Open(const std::string& port, int baud) {
  Close();
  const std::string path =
      (port.rfind("\\\\.\\", 0) == 0)
          ? port
          : "\\\\.\\" + port;  // COM10 and above need this form
  HANDLE h = CreateFileA(path.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr,
                         OPEN_EXISTING, 0, nullptr);
  if (h == INVALID_HANDLE_VALUE)
    throw std::runtime_error("cannot open " + port);
  DCB dcb{};
  dcb.DCBlength = sizeof(dcb);
  if (!GetCommState(h, &dcb)) {
    CloseHandle(h);
    throw std::runtime_error("GetCommState failed on " + port);
  }
  dcb.BaudRate = static_cast<DWORD>(baud);
  dcb.ByteSize = 8;
  dcb.Parity = NOPARITY;
  dcb.StopBits = ONESTOPBIT;
  dcb.fBinary = TRUE;
  dcb.fParity = FALSE;
  dcb.fOutxCtsFlow = dcb.fOutxDsrFlow = FALSE;
  dcb.fDtrControl = DTR_CONTROL_ENABLE;
  dcb.fRtsControl = RTS_CONTROL_ENABLE;
  dcb.fOutX = dcb.fInX = FALSE;
  if (!SetCommState(h, &dcb)) {
    CloseHandle(h);
    throw std::runtime_error("the driver refused " + std::to_string(baud) +
                             " baud on " + port);
  }
  handle_ = h;
  FlushInput();
  // The FTDI latency timer (default 16 ms) is set in Device Manager: port,
  // Advanced, Latency Timer = 1.
}

void SerialPort::Close() {
  if (handle_) {
    CloseHandle(static_cast<HANDLE>(handle_));
    handle_ = nullptr;
  }
}

void SerialPort::Write(const std::uint8_t* data, std::size_t len) {
  DWORD done = 0;
  if (!WriteFile(static_cast<HANDLE>(handle_), data, static_cast<DWORD>(len),
                 &done, nullptr) ||
      done != len)
    throw std::runtime_error("serial write failed");
}

std::size_t SerialPort::Read(std::uint8_t* data, std::size_t max_len,
                             int timeout_us) {
  COMMTIMEOUTS t{};
  t.ReadIntervalTimeout = MAXDWORD;  // return as soon as any byte is there,
  t.ReadTotalTimeoutMultiplier =
      MAXDWORD;  // otherwise wait at most the constant below
  t.ReadTotalTimeoutConstant =
      static_cast<DWORD>(timeout_us < 1000 ? 1 : timeout_us / 1000);
  SetCommTimeouts(static_cast<HANDLE>(handle_), &t);
  DWORD got = 0;
  if (!ReadFile(static_cast<HANDLE>(handle_), data, static_cast<DWORD>(max_len),
                &got, nullptr))
    throw std::runtime_error("serial read failed");
  return got;
}

void SerialPort::FlushInput() {
  PurgeComm(static_cast<HANDLE>(handle_), PURGE_RXCLEAR);
}

std::string SerialPort::FindPort() {
  throw std::runtime_error("give the port name, for example COM5");
}

}  // namespace g1motor

#else
// ---------------------------------------------------------------- Linux and
// macOS
#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

#if defined(__linux__)
#include <asm/termbits.h>  // struct termios2, BOTHER, TCGETS2 (cannot be mixed with <termios.h>)
#include <glob.h>
#include <linux/serial.h>
#elif defined(__APPLE__)
#include <IOKit/serial/ioss.h>  // IOSSIOSPEED
#include <termios.h>
#else
#error "unsupported platform"
#endif

namespace g1motor {

SerialPort::~SerialPort() { Close(); }
bool SerialPort::is_open() const { return fd_ >= 0; }

void SerialPort::Open(const std::string& port, int baud) {
  Close();
  const int fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0)
    throw std::runtime_error("cannot open " + port + ": " +
                             std::strerror(errno));
#if defined(__linux__)
  struct termios2 tio{};
  if (ioctl(fd, TCGETS2, &tio) < 0) {
    ::close(fd);
    throw std::runtime_error("TCGETS2 failed on " + port);
  }
  tio.c_iflag = 0;
  tio.c_oflag = 0;
  tio.c_lflag = 0;
  tio.c_cflag = CS8 | CREAD | CLOCAL | BOTHER;
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 0;
  tio.c_ispeed = tio.c_ospeed = static_cast<speed_t>(baud);
  if (ioctl(fd, TCSETS2, &tio) < 0 || ioctl(fd, TCGETS2, &tio) < 0 ||
      static_cast<int>(tio.c_ospeed) != baud) {
    ::close(fd);
    throw std::runtime_error("the driver refused " + std::to_string(baud) +
                             " baud on " + port);
  }
  struct serial_struct ss{};  // FTDI latency timer 16 ms -> 1 ms
  if (ioctl(fd, TIOCGSERIAL, &ss) == 0) {
    ss.flags |= ASYNC_LOW_LATENCY;
    ioctl(fd, TIOCSSERIAL, &ss);
  }
#else
  struct termios tio{};
  if (tcgetattr(fd, &tio) < 0) {
    ::close(fd);
    throw std::runtime_error("tcgetattr failed on " + port);
  }
  cfmakeraw(&tio);
  tio.c_cflag |= CLOCAL | CREAD;
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 0;
  if (tcsetattr(fd, TCSANOW, &tio) < 0) {
    ::close(fd);
    throw std::runtime_error("tcsetattr failed on " + port);
  }
  speed_t speed = static_cast<speed_t>(
      baud);  // must come after tcsetattr, which resets the rate
  if (ioctl(fd, IOSSIOSPEED, &speed) < 0) {
    ::close(fd);
    throw std::runtime_error(
        "the driver refused " + std::to_string(baud) + " baud on " + port +
        " (Apple's built-in FTDI driver is reported to stop at about 3 Mbaud)");
  }
#endif
  fd_ = fd;
  usleep(100000);  // let bytes received before the rate change drain
  FlushInput();
}

void SerialPort::Close() {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

void SerialPort::Write(const std::uint8_t* data, std::size_t len) {
  std::size_t done = 0;
  while (done < len) {
    const ssize_t n = ::write(fd_, data + done, len - done);
    if (n > 0) {
      done += static_cast<std::size_t>(n);
      continue;
    }
    if (n < 0 && (errno == EAGAIN || errno == EINTR)) {
      struct pollfd p{fd_, POLLOUT, 0};
      poll(&p, 1, 10);
      continue;
    }
    throw std::runtime_error(std::string("serial write failed: ") +
                             std::strerror(errno));
  }
}

std::size_t SerialPort::Read(std::uint8_t* data, std::size_t max_len,
                             int timeout_us) {
  struct pollfd p{fd_, POLLIN, 0};
  struct timespec ts{timeout_us / 1000000, (timeout_us % 1000000) * 1000L};
#if defined(__linux__)
  const int ready = ppoll(&p, 1, &ts, nullptr);
#else
  (void)ts;
  const int ready = poll(&p, 1, timeout_us < 1000 ? 1 : timeout_us / 1000);
#endif
  if (ready <= 0) return 0;
  const ssize_t n = ::read(fd_, data, max_len);
  return n > 0 ? static_cast<std::size_t>(n) : 0;
}

void SerialPort::FlushInput() {
#if defined(__linux__)
  ioctl(fd_, TCFLSH, TCIFLUSH);
#else
  tcflush(fd_, TCIFLUSH);
#endif
}

std::string SerialPort::FindPort() {
#if defined(__linux__)
  glob_t g{};
  std::vector<std::string> found;
  if (glob("/dev/serial/by-id/*FTDI*", 0, nullptr, &g) == 0)
    for (std::size_t i = 0; i < g.gl_pathc; ++i)
      found.emplace_back(g.gl_pathv[i]);
  globfree(&g);
  if (found.size() == 1) return found[0];
  if (found.empty())
    throw std::runtime_error(
        "no FTDI serial adapter found; give the port name");
  throw std::runtime_error("several FTDI adapters found; give the port name");
#else
  throw std::runtime_error(
      "give the port name, for example /dev/cu.usbserial-XXXX");
#endif
}

}  // namespace g1motor
#endif

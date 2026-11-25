#include "g1motor/protocol.h"

#include <stdexcept>
#include <string>

namespace g1motor {
namespace {

/// Writes a little-endian int16 after checking that the value fits.
void put16(std::uint8_t* p, int value, const char* name) {
  if (value < -32768 || value > 32767) {
    throw std::out_of_range(std::string(name) + " does not fit a 16-bit field");
  }
  const auto u = static_cast<std::uint16_t>(static_cast<std::int16_t>(value));
  p[0] = static_cast<std::uint8_t>(u & 0xFF);
  p[1] = static_cast<std::uint8_t>(u >> 8);
}

/// Writes a little-endian uint32.
void put32(std::uint8_t* p, std::uint32_t value) {
  for (int i = 0; i < 4; ++i) {
    p[i] = static_cast<std::uint8_t>((value >> (8 * i)) & 0xFF);
  }
}

/// @return The little-endian int16 at `p`.
int get16(const std::uint8_t* p) {
  return static_cast<std::int16_t>(
      static_cast<std::uint16_t>(p[0] | (p[1] << 8)));
}

/// @return The little-endian uint32 at `p`.
std::uint32_t get32(const std::uint8_t* p) {
  return static_cast<std::uint32_t>(p[0]) |
         (static_cast<std::uint32_t>(p[1]) << 8) |
         (static_cast<std::uint32_t>(p[2]) << 16) |
         (static_cast<std::uint32_t>(p[3]) << 24);
}

}  // namespace

std::uint32_t crc32_core(const std::uint8_t* data, std::size_t len) {
  if (len % 4 != 0) {
    throw std::invalid_argument("crc32_core needs a multiple of 4 bytes");
  }
  std::uint32_t crc = 0xFFFFFFFFu;
  for (std::size_t i = 0; i < len; i += 4) {
    const std::uint32_t word = get32(data + i);
    for (std::uint32_t bit = 0x80000000u; bit != 0; bit >>= 1) {
      crc = (crc & 0x80000000u) ? (crc << 1) ^ 0x04C11DB7u : (crc << 1);
      if (word & bit) crc ^= 0x04C11DB7u;
    }
  }
  return crc;
}

std::array<std::uint8_t, CMD_LEN> MotorCmd::encode() const {
  if (id < 0 || id >= BROADCAST_ID) {
    throw std::out_of_range("motor id out of range 0-14");
  }
  std::array<std::uint8_t, CMD_LEN> frame{};
  frame[0] = 0xFE;
  frame[1] = 0xEE;
  frame[2] =
      static_cast<std::uint8_t>(id | ((mode & 0x07) << 4) | (flag ? 0x80 : 0));
  frame[3] = byte3;
  put16(&frame[4], tau, "tau");
  put16(&frame[6], vel, "vel");
  put32(&frame[8], static_cast<std::uint32_t>(pos));
  put16(&frame[12], kp, "kp");
  put16(&frame[14], kd, "kd");
  put32(&frame[16], crc32_core(frame.data(), 16));
  return frame;
}

std::optional<MotorCmd> MotorCmd::decode(const std::uint8_t* frame,
                                         std::size_t len) {
  if (len != CMD_LEN || frame[0] != 0xFE || frame[1] != 0xEE) {
    return std::nullopt;
  }
  if (get32(frame + 16) != crc32_core(frame, 16)) return std::nullopt;
  MotorCmd cmd;
  cmd.id = frame[2] & 0x0F;
  cmd.mode = (frame[2] >> 4) & 0x07;
  cmd.flag = (frame[2] & 0x80) != 0;
  cmd.byte3 = frame[3];
  cmd.tau = get16(frame + 4);
  cmd.vel = get16(frame + 6);
  cmd.pos = static_cast<std::int32_t>(get32(frame + 8));
  cmd.kp = get16(frame + 12);
  cmd.kd = get16(frame + 14);
  return cmd;
}

std::optional<MotorState> MotorState::decode(const std::uint8_t* frame,
                                             std::size_t len) {
  if (len != REPLY_LEN || frame[0] != 0xFC || frame[1] != 0xEE) {
    return std::nullopt;
  }
  // The reply checksum covers bytes 2-21; the header is left out.
  if (get32(frame + 22) != crc32_core(frame + 2, 20)) return std::nullopt;
  MotorState state;
  state.id = frame[2] & 0x0F;
  state.mode = (frame[2] >> 4) & 0x07;
  state.temp_coil = frame[3];
  state.temp_shell = frame[4];
  state.voltage_raw = frame[5];
  state.tau = get16(frame + 6);
  state.vel = get16(frame + 8);
  state.pos = static_cast<std::int32_t>(get32(frame + 10));
  state.error = get32(frame + 14);
  state.word18 = frame[18] | (frame[19] << 8);
  state.byte20 = frame[20];
  state.byte21 = frame[21];
  return state;
}

std::optional<MotorState> find_reply(const std::uint8_t* buffer,
                                     std::size_t len) {
  for (std::size_t i = 0; i + REPLY_LEN <= len; ++i) {
    if (buffer[i] != 0xFC || buffer[i + 1] != 0xEE) continue;
    if (auto state = MotorState::decode(buffer + i, REPLY_LEN)) return state;
  }
  return std::nullopt;
}

}  // namespace g1motor

#include "g1motor/protocol.hpp"

#include <stdexcept>
#include <string>

namespace g1motor {
namespace {

void put16(std::uint8_t* p, int v, const char* name) {
    if (v < -32768 || v > 32767) throw std::out_of_range(std::string(name) + " does not fit a 16-bit field");
    const auto u = static_cast<std::uint16_t>(static_cast<std::int16_t>(v));
    p[0] = static_cast<std::uint8_t>(u & 0xFF);
    p[1] = static_cast<std::uint8_t>(u >> 8);
}
void put32(std::uint8_t* p, std::uint32_t u) {
    for (int i = 0; i < 4; ++i) p[i] = static_cast<std::uint8_t>((u >> (8 * i)) & 0xFF);
}
int get16(const std::uint8_t* p) { return static_cast<std::int16_t>(static_cast<std::uint16_t>(p[0] | (p[1] << 8))); }
std::uint32_t get32(const std::uint8_t* p) {
    return static_cast<std::uint32_t>(p[0]) | (static_cast<std::uint32_t>(p[1]) << 8) |
           (static_cast<std::uint32_t>(p[2]) << 16) | (static_cast<std::uint32_t>(p[3]) << 24);
}

}  // namespace

std::uint32_t crc32_core(const std::uint8_t* data, std::size_t len) {
    if (len % 4) throw std::invalid_argument("crc32_core needs a multiple of 4 bytes");
    std::uint32_t crc = 0xFFFFFFFFu;
    for (std::size_t i = 0; i < len; i += 4) {
        const std::uint32_t word = get32(data + i);
        for (std::uint32_t bit = 0x80000000u; bit; bit >>= 1) {
            crc = (crc & 0x80000000u) ? (crc << 1) ^ 0x04C11DB7u : (crc << 1);
            if (word & bit) crc ^= 0x04C11DB7u;
        }
    }
    return crc;
}

std::array<std::uint8_t, kCmdLen> MotorCmd::encode() const {
    if (id < 0 || id >= kBroadcastId) throw std::out_of_range("motor id out of range 0-14");
    std::array<std::uint8_t, kCmdLen> f{};
    f[0] = 0xFE;
    f[1] = 0xEE;
    f[2] = static_cast<std::uint8_t>(id | ((mode & 0x07) << 4) | (flag ? 0x80 : 0));
    f[3] = byte3;
    put16(&f[4], tau, "tau");
    put16(&f[6], vel, "vel");
    put32(&f[8], static_cast<std::uint32_t>(pos));
    put16(&f[12], kp, "kp");
    put16(&f[14], kd, "kd");
    put32(&f[16], crc32_core(f.data(), 16));
    return f;
}

std::optional<MotorCmd> MotorCmd::decode(const std::uint8_t* f, std::size_t len) {
    if (len != kCmdLen || f[0] != 0xFE || f[1] != 0xEE) return std::nullopt;
    if (get32(f + 16) != crc32_core(f, 16)) return std::nullopt;
    MotorCmd c;
    c.id = f[2] & 0x0F;
    c.mode = (f[2] >> 4) & 0x07;
    c.flag = (f[2] & 0x80) != 0;
    c.byte3 = f[3];
    c.tau = get16(f + 4);
    c.vel = get16(f + 6);
    c.pos = static_cast<std::int32_t>(get32(f + 8));
    c.kp = get16(f + 12);
    c.kd = get16(f + 14);
    return c;
}

std::optional<MotorState> MotorState::decode(const std::uint8_t* f, std::size_t len) {
    if (len != kReplyLen || f[0] != 0xFC || f[1] != 0xEE) return std::nullopt;
    if (get32(f + 22) != crc32_core(f + 2, 20)) return std::nullopt;   // the header is not covered
    MotorState s;
    s.id = f[2] & 0x0F;
    s.mode = (f[2] >> 4) & 0x07;
    s.temp_coil = f[3];
    s.temp_shell = f[4];
    s.voltage_raw = f[5];
    s.tau = get16(f + 6);
    s.vel = get16(f + 8);
    s.pos = static_cast<std::int32_t>(get32(f + 10));
    s.error = get32(f + 14);
    s.word18 = f[18] | (f[19] << 8);
    s.byte20 = f[20];
    s.byte21 = f[21];
    return s;
}

std::optional<MotorState> find_reply(const std::uint8_t* b, std::size_t len) {
    for (std::size_t i = 0; i + kReplyLen <= len; ++i) {
        if (b[i] == 0xFC && b[i + 1] == 0xEE) {
            if (auto s = MotorState::decode(b + i, kReplyLen)) return s;
        }
    }
    return std::nullopt;
}

}  // namespace g1motor

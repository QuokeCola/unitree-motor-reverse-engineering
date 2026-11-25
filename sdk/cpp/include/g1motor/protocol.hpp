// Wire format of the Unitree G1 joint motor bus (unofficial, reverse engineered).
// Everything is in raw counts exactly as on the wire. See the protocol guide for the layout.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace g1motor {

constexpr int kBaud = 6000000;
constexpr std::size_t kCmdLen = 20;
constexpr std::size_t kReplyLen = 26;
constexpr int kBroadcastId = 15;   // broadcast in the public GO-M8010-6 protocol; never polled here

enum Mode : int {
    kIdle = 0,      // commands ignored; joint passively braked
    kRun = 1,       // hybrid torque / position / velocity command
    kRestart = 7,   // motor restarts: silent about 0.3 s, fault cleared, position re-referenced
    // 3 and 5: accepted, commands ignored, joint braked. 4: accepted, commands ignored, joint free.
    // 2: ignored. 6: sets the position zero with the calibration payload; not sent by this library.
};

// CRC-32, polynomial 0x04C11DB7, init 0xFFFFFFFF, no reflection, no final XOR, over 32-bit
// little-endian words fed most-significant bit first. len must be a multiple of 4.
std::uint32_t crc32_core(const std::uint8_t* data, std::size_t len);

// One command frame. torque = tau + kp * (pos - measured) + kd * (vel - measured).
struct MotorCmd {
    int id = 0;
    int mode = kIdle;
    int tau = 0;              // feed-forward torque, int16
    int vel = 0;              // velocity target, int16
    std::int32_t pos = 0;     // position target, int32
    int kp = 0;               // position gain, int16
    int kd = 0;               // damping gain, int16; damping torque = 0.005 * kd * velocity reading
    bool flag = false;        // top bit of the id/mode byte; set by the G1 controller, not required
    std::uint8_t byte3 = 0;   // sequence byte; the controller toggles 0x80 / 0x90, not required

    // Throws std::out_of_range if a field does not fit or the id is not 0-14.
    std::array<std::uint8_t, kCmdLen> encode() const;
    // Empty if the header or checksum is wrong.
    static std::optional<MotorCmd> decode(const std::uint8_t* frame, std::size_t len);
};

// One reply frame.
struct MotorState {
    int id = 0;
    int mode = 0;               // mode the motor is actually in
    int tau = 0;                // net applied torque, int16
    int vel = 0;                // velocity, int16 (1 count = 128 position counts per second)
    std::int32_t pos = 0;       // position, multi-turn; 32768 counts per rotor turn
    int temp_coil = 0;          // winding temperature, deg C
    int temp_shell = 0;         // shell temperature, deg C
    int voltage_raw = 0;        // bus voltage, 0.5 V per count
    std::uint32_t error = 0;    // status word; 0 in normal operation, 0x8 = supply under-voltage seen
    int word18 = 0;             // unknown
    int byte20 = 0;             // unknown
    int byte21 = 0;             // unknown

    double voltage() const { return voltage_raw * 0.5; }
    bool fault() const { return error != 0; }
    static std::optional<MotorState> decode(const std::uint8_t* frame, std::size_t len);
};

// First valid reply inside a receive buffer (skips echo and noise).
std::optional<MotorState> find_reply(const std::uint8_t* buffer, std::size_t len);

}  // namespace g1motor

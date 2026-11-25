/// @file protocol_test.cc
/// @brief Frame tests against frames captured from a G1 arm bus. No hardware.

#include <algorithm>
#include <stdexcept>

#include "g1motor/g1motor.h"
#include "test_util.h"

namespace {

using g1motor::crc32_core;
using g1motor::find_reply;
using g1motor::MotorCmd;
using g1motor::MotorState;
using g1motor_test::hex;

/// Run mode, zero torque, id 0 (robot).
const char* const CMD_ZERO_TORQUE = "feee90800000000000000000000000009b1e4106";
/// Run mode, kd 25, id 0 (robot damping).
const char* const CMD_DAMPING = "feee9080000000000000000000001900541ed614";
/// Idle poll, id 0.
const char* const CMD_IDLE = "feee000000000000000000000000000080462079";
/// Hold 25,032 with kp 450, kd 10, id 0 (robot standing).
const char* const CMD_HOLD = "feee908000000000c8610000c2010a0071dc7c64";
/// Restart, id 0 (robot, after calibration).
const char* const CMD_RESTART = "feee7000000000000000000000000000378838d7";
/// Reply of robot id 2 in run mode.
const char* const REPLY_ROBOT =
    "fcee12242361fffff8ffc63dfdff00000000cc0b00c82563ad8c";
/// Reply of bench id 4 in idle on 24 V.
const char* const REPLY_BENCH =
    "fcee041e1c30fbfff9ff1b08010000000000c00b000057221ecf";
/// Reply of bench id 4 with status 0x8.
const char* const REPLY_FAULT =
    "fcee042626300000fdffc15d010008000000a80b0000a190371a";

/// @return True if the frame equals the bytes given as hex.
bool same(const std::array<std::uint8_t, g1motor::CMD_LEN>& frame,
          const char* expected_hex) {
  const auto expected = hex(expected_hex);
  return expected.size() == frame.size() &&
         std::equal(frame.begin(), frame.end(), expected.begin());
}

/// @return A run-mode command for id 0 as the G1 controller sends it.
MotorCmd run_cmd() {
  MotorCmd cmd;
  cmd.mode = g1motor::MODE_RUN;
  cmd.flag = true;
  cmd.byte3 = 0x80;
  return cmd;
}

/// Checksums of a captured command and reply.
void test_checksum() {
  const auto command = hex(CMD_ZERO_TORQUE);
  CHECK(crc32_core(command.data(), 16) == 0x06411E9Bu);
  const auto reply = hex(REPLY_ROBOT);
  CHECK(crc32_core(reply.data() + 2, 20) == 0x8CAD6325u);
}

/// Encoded commands equal the captured frames.
void test_encode_matches_capture() {
  CHECK(same(MotorCmd().encode(), CMD_IDLE));
  CHECK(same(run_cmd().encode(), CMD_ZERO_TORQUE));
  MotorCmd damping = run_cmd();
  damping.kd = 25;
  CHECK(same(damping.encode(), CMD_DAMPING));
  MotorCmd hold = run_cmd();
  hold.pos = 25032;
  hold.kp = 450;
  hold.kd = 10;
  CHECK(same(hold.encode(), CMD_HOLD));
  MotorCmd restart;
  restart.mode = g1motor::MODE_RESTART;
  CHECK(same(restart.encode(), CMD_RESTART));
}

/// A command survives encode and decode.
void test_command_round_trip() {
  MotorCmd cmd;
  cmd.id = 6;
  cmd.mode = g1motor::MODE_RUN;
  cmd.flag = true;
  cmd.byte3 = 0x90;
  cmd.tau = -12;
  cmd.vel = 345;
  cmd.pos = -123456;
  cmd.kp = 300;
  cmd.kd = 80;
  const auto frame = cmd.encode();
  const auto back = MotorCmd::decode(frame.data(), frame.size());
  CHECK(back && back->id == 6 && back->mode == g1motor::MODE_RUN);
  CHECK(back && back->flag && back->byte3 == 0x90 && back->tau == -12);
  CHECK(back && back->vel == 345 && back->pos == -123456);
  CHECK(back && back->kp == 300 && back->kd == 80);
}

/// Fields of captured robot and bench replies.
void test_decode_reply() {
  const auto robot = hex(REPLY_ROBOT);
  const auto state = MotorState::decode(robot.data(), robot.size());
  CHECK(state && state->id == 2 && state->mode == 1);
  CHECK(state && state->pos == -180794 && state->vel == -8);
  CHECK(state && state->tau == -1 && state->temp_coil == 36);
  CHECK(state && state->temp_shell == 35 && state->voltage() == 48.5);
  CHECK(state && !state->fault());
  const auto bench_bytes = hex(REPLY_BENCH);
  const auto bench = MotorState::decode(bench_bytes.data(), bench_bytes.size());
  CHECK(bench && bench->id == 4 && bench->mode == g1motor::MODE_IDLE);
  CHECK(bench && bench->pos == 67611 && bench->voltage() == 24.0);
}

/// A captured reply with status 0x8 reports a fault.
void test_fault_word() {
  const auto bytes = hex(REPLY_FAULT);
  const auto state = MotorState::decode(bytes.data(), bytes.size());
  CHECK(state && state->fault() && state->error == 0x8u);
  CHECK(state && state->mode == g1motor::MODE_IDLE);
}

/// Corrupt or short frames decode to nothing.
void test_bad_frames_rejected() {
  const auto reply = hex(REPLY_ROBOT);
  auto broken = reply;
  broken[10] ^= 1;
  CHECK(!MotorState::decode(broken.data(), broken.size()));
  CHECK(!MotorState::decode(reply.data(), reply.size() - 1));
  auto command = hex(CMD_IDLE);
  command.back() = 0;
  CHECK(!MotorCmd::decode(command.data(), command.size()));
}

/// A reply is found after echo and noise.
void test_find_reply_skips_echo_and_noise() {
  auto buffer = hex(CMD_IDLE);
  const auto noise = hex("fcee00");
  const auto reply = hex(REPLY_BENCH);
  buffer.insert(buffer.end(), noise.begin(), noise.end());
  buffer.insert(buffer.end(), reply.begin(), reply.end());
  const auto found = find_reply(buffer.data(), buffer.size());
  CHECK(found && found->id == 4);
  const auto command = hex(CMD_IDLE);
  CHECK(!find_reply(command.data(), command.size()));
}

/// Out-of-range fields and ids throw.
void test_field_range_checked() {
  bool threw = false;
  try {
    MotorCmd cmd;
    cmd.id = 4;
    cmd.kp = 40000;
    cmd.encode();
  } catch (const std::out_of_range&) {
    threw = true;
  }
  CHECK(threw);
  threw = false;
  try {
    MotorCmd cmd;
    cmd.id = 15;
    cmd.encode();
  } catch (const std::out_of_range&) {
    threw = true;
  }
  CHECK(threw);
}

}  // namespace

/// Runs every test and returns 1 if any check failed.
int main() {
  test_checksum();
  test_encode_matches_capture();
  test_command_round_trip();
  test_decode_reply();
  test_fault_word();
  test_bad_frames_rejected();
  test_find_reply_skips_echo_and_noise();
  test_field_range_checked();
  return g1motor_test::finish("protocol_test");
}

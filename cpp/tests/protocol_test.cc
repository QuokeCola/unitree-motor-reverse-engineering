/// @file protocol_test.cc
/// @brief Frame and joint tests against frames captured from a G1 arm bus.
///
/// No hardware needed.

#include <algorithm>
#include <stdexcept>

#include "g1motor/g1motor.h"
#include "test_util.h"

namespace {

using g1motor::Crc32Core;
using g1motor::FindReply;
using g1motor::Joint;
using g1motor::MotorCmd;
using g1motor::MotorState;
using g1motor_test::Hex;

/// @return True if the frame equals the bytes given as hex.
bool Same(const std::array<std::uint8_t, g1motor::kCmdLen>& frame,
          const std::string& hex) {
  const auto expected = Hex(hex);
  return expected.size() == frame.size() &&
         std::equal(frame.begin(), frame.end(), expected.begin());
}

/// Checksums of a captured command and reply.
void TestChecksum() {
  const auto command = Hex("feee90800000000000000000000000009b1e4106");
  CHECK(Crc32Core(command.data(), 16) == 0x06411E9Bu);
  const auto reply =
      Hex("fcee12242361fffff8ffc63dfdff00000000cc0b00c82563ad8c");
  CHECK(Crc32Core(reply.data() + 2, 20) == 0x8CAD6325u);
}

/// Encoded commands equal the captured frames.
void TestEncodeMatchesCapture() {
  MotorCmd idle;
  CHECK(Same(idle.Encode(), "feee000000000000000000000000000080462079"));
  MotorCmd run;
  run.mode = g1motor::kRun;
  run.flag = true;
  run.byte3 = 0x80;
  CHECK(Same(run.Encode(), "feee90800000000000000000000000009b1e4106"));
  MotorCmd damping = run;
  damping.kd = 25;
  CHECK(Same(damping.Encode(), "feee9080000000000000000000001900541ed614"));
  MotorCmd hold = run;
  hold.pos = 25032;
  hold.kp = 450;
  hold.kd = 10;
  CHECK(Same(hold.Encode(), "feee908000000000c8610000c2010a0071dc7c64"));
  MotorCmd restart;
  restart.mode = g1motor::kRestart;
  CHECK(Same(restart.Encode(), "feee7000000000000000000000000000378838d7"));
}

/// A command survives encode and decode.
void TestCommandRoundTrip() {
  MotorCmd cmd;
  cmd.id = 6;
  cmd.mode = g1motor::kRun;
  cmd.flag = true;
  cmd.byte3 = 0x90;
  cmd.tau = -12;
  cmd.vel = 345;
  cmd.pos = -123456;
  cmd.kp = 300;
  cmd.kd = 80;
  const auto frame = cmd.Encode();
  const auto back = MotorCmd::Decode(frame.data(), frame.size());
  CHECK(back && back->id == 6 && back->mode == g1motor::kRun && back->flag);
  CHECK(back && back->byte3 == 0x90 && back->tau == -12 && back->vel == 345);
  CHECK(back && back->pos == -123456 && back->kp == 300 && back->kd == 80);
}

/// Fields of captured replies, with and without a fault.
void TestDecodeReply() {
  const auto reply =
      Hex("fcee12242361fffff8ffc63dfdff00000000cc0b00c82563ad8c");
  const auto state = MotorState::Decode(reply.data(), reply.size());
  CHECK(state && state->id == 2 && state->mode == 1);
  CHECK(state && state->pos == -180794 && state->vel == -8);
  CHECK(state && state->tau == -1 && state->temp_coil == 36);
  CHECK(state && state->temp_shell == 35 && state->voltage() == 48.5);
  CHECK(state && !state->fault());
  const auto faulted =
      Hex("fcee042626300000fdffc15d010008000000a80b0000a190371a");
  const auto fault = MotorState::Decode(faulted.data(), faulted.size());
  CHECK(fault && fault->fault() && fault->error == 0x8u);
  CHECK(fault && fault->mode == g1motor::kIdle && fault->voltage() == 24.0);
}

/// Corrupt or short frames are rejected; a reply is found after noise.
void TestBadFramesRejected() {
  const auto reply =
      Hex("fcee12242361fffff8ffc63dfdff00000000cc0b00c82563ad8c");
  auto broken = reply;
  broken[10] ^= 1;
  CHECK(!MotorState::Decode(broken.data(), broken.size()));
  CHECK(!MotorState::Decode(reply.data(), reply.size() - 1));
  auto noisy = Hex("feee000000000000000000000000000080462079fcee00");
  noisy.insert(noisy.end(), reply.begin(), reply.end());
  const auto found = FindReply(noisy.data(), noisy.size());
  CHECK(found && found->id == 2);
  const auto command = Hex("feee90800000000000000000000000009b1e4106");
  CHECK(!FindReply(command.data(), command.size()));
}

/// Out-of-range fields and ids throw.
void TestFieldRangeChecked() {
  bool threw = false;
  try {
    MotorCmd cmd;
    cmd.id = 4;
    cmd.kp = 40000;
    cmd.Encode();
  } catch (const std::out_of_range&) {
    threw = true;
  }
  CHECK(threw);
  threw = false;
  try {
    MotorCmd cmd;
    cmd.id = 15;
    cmd.Encode();
  } catch (const std::out_of_range&) {
    threw = true;
  }
  CHECK(threw);
}

/// Joint: whole-turn re-referencing, angles, clamping.
void TestJoint() {
  Joint wrist(25, -188529, 236437);
  const auto zero = wrist.zero;
  CHECK(Joint(16).turn_counts() == 524288 && wrist.turn_counts() == 819200);
  CHECK(wrist.Sync(830810) == 1);
  CHECK(*wrist.low == -188529 + 819200 && *wrist.high == 236437 + 819200);
  CHECK(wrist.zero == zero + 819200 && wrist.Sync(830810) == 0);

  Joint roll(16, -94259, 236635);
  CHECK(roll.Counts(roll.Angle(123456)) == 123456);
  bool threw = false;
  try {
    roll.Sync(236635 + 100000);
  } catch (const std::runtime_error&) {
    threw = true;
  }
  CHECK(threw);

  Joint limited(25, 0, 400000);
  limited.margin = 40000;
  CHECK(limited.Clamp(-5) == 40000 && limited.Clamp(999999) == 360000);
}

}  // namespace

/// Runs every test and returns 1 if any check failed.
int main() {
  TestChecksum();
  TestEncodeMatchesCapture();
  TestCommandRoundTrip();
  TestDecodeReply();
  TestBadFramesRejected();
  TestFieldRangeChecked();
  TestJoint();
  return g1motor_test::Finish("protocol_test");
}

/// @file joint_test.cc
/// @brief Tests of Joint: whole-turn re-referencing, angles, clamping.
///
/// No hardware.

#include <cmath>
#include <stdexcept>

#include "g1motor/g1motor.h"
#include "test_util.h"

namespace {

using g1motor::Joint;

/// Value of pi.
constexpr double PI = 3.14159265358979323846;

/// Counts per output turn for 16:1 and 25:1.
void test_turn_counts() {
  CHECK(Joint(16).turn_counts() == 524288);
  CHECK(Joint(25).turn_counts() == 819200);
}

/// sync() follows a reading shifted by one output turn.
void test_sync_follows_whole_turn_shift() {
  // End stops measured before a restart; the reading then came back one
  // output turn higher (bench id 6).
  Joint joint(25, -188529, 236437);
  const auto zero = joint.zero;
  CHECK(joint.sync(830810) == 1);
  CHECK(*joint.low == -188529 + 819200);
  CHECK(*joint.high == 236437 + 819200);
  CHECK(joint.zero == zero + 819200);
  CHECK(joint.sync(830810) == 0);
}

/// sync() rejects a reading beyond the end stops.
void test_sync_rejects_reading_outside_end_stops() {
  Joint joint(16, -94259, 236635);
  bool threw = false;
  try {
    joint.sync(236635 + 100000);
  } catch (const std::invalid_argument&) {
    threw = true;
  }
  CHECK(threw);
}

/// Angle and counts convert back and forth; travel in degrees.
void test_angle_round_trip_and_travel() {
  Joint joint(16, -94259, 236635);
  CHECK(joint.angle(joint.zero) == 0);
  CHECK(joint.counts(joint.angle(123456)) == 123456);
  CHECK(std::fabs(*joint.travel() * 180 / PI - 227.2) < 0.1);
  CHECK(std::fabs(joint.angle(joint.zero + 524288) - 2 * PI) < 1e-9);
}

/// clamp() keeps the margin from each end stop.
void test_clamp_uses_margin() {
  Joint joint(25, 0, 400000);
  joint.margin = 40000;
  CHECK(joint.clamp(-5) == 40000);
  CHECK(joint.clamp(999999) == 360000);
  CHECK(joint.clamp(123456.4) == 123456);
}

/// Output speed for the 24 V speed ceiling on bench id 4.
void test_speed() { CHECK(std::fabs(Joint(16).speed(9960) - 15.28) < 0.02); }

}  // namespace

/// Runs every test and returns 1 if any check failed.
int main() {
  test_turn_counts();
  test_sync_follows_whole_turn_shift();
  test_sync_rejects_reading_outside_end_stops();
  test_angle_round_trip_and_travel();
  test_clamp_uses_margin();
  test_speed();
  return g1motor_test::finish("joint_test");
}

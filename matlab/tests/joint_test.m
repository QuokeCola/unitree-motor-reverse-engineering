%> @file joint_test.m
%> @brief Tests of Joint: whole-turn re-referencing, angles, clamping.
%>
%> No hardware. Errors on the first failed check.

%> @brief Runs every test.
function joint_test()
addpath(fullfile(fileparts(mfilename('fullpath')), '..'));
test_turn_counts();
test_sync_follows_whole_turn_shift();
test_sync_rejects_reading_outside_end_stops();
test_angle_round_trip_and_travel();
test_clamp_uses_margin();
test_speed();
disp('joint_test: all checks passed');
end

%> @brief Counts per output turn for 16:1 and 25:1.
function test_turn_counts()
assert(g1motor.Joint(16).turn_counts() == 524288);
assert(g1motor.Joint(25).turn_counts() == 819200);
end

%> @brief sync() follows a reading shifted by one output turn.
function test_sync_follows_whole_turn_shift()
% End stops measured before a restart; the reading then came back one
% output turn higher (bench id 6).
joint = g1motor.Joint(25, -188529, 236437);
zero = joint.zero;
assert(joint.sync(830810) == 1);
assert(joint.low == -188529 + 819200);
assert(joint.high == 236437 + 819200);
assert(joint.zero == zero + 819200);
assert(joint.sync(830810) == 0);
end

%> @brief sync() rejects a reading beyond the end stops.
function test_sync_rejects_reading_outside_end_stops()
joint = g1motor.Joint(16, -94259, 236635);
threw = false;
try
    joint.sync(236635 + 100000);
catch
    threw = true;
end
assert(threw);
end

%> @brief Angle and counts convert back and forth; travel in degrees.
function test_angle_round_trip_and_travel()
joint = g1motor.Joint(16, -94259, 236635);
assert(joint.angle(joint.zero) == 0);
assert(joint.counts(joint.angle(123456)) == 123456);
assert(abs(rad2deg(joint.travel()) - 227.2) < 0.1);
assert(abs(joint.angle(joint.zero + 524288) - 2 * pi) < 1e-9);
end

%> @brief clamp() keeps the margin from each end stop.
function test_clamp_uses_margin()
joint = g1motor.Joint(25, 0, 400000);
joint.margin = 40000;
assert(joint.clamp(-5) == 40000);
assert(joint.clamp(999999) == 360000);
assert(joint.clamp(123456.4) == 123456);
end

%> @brief Output speed for the 24 V speed ceiling on bench id 4.
function test_speed()
assert(abs(g1motor.Joint(16).speed(9960) - 15.28) < 0.02);
end

%> @file protocol_test.m
%> @brief Frame tests against frames captured from a G1 arm bus. No hardware.
%>
%> Errors on the first failed check.

%> @brief Runs every test.
function protocol_test()
addpath(fullfile(fileparts(mfilename('fullpath')), '..'));
test_checksum();
test_encode_matches_capture();
test_command_round_trip();
test_decode_reply();
test_fault_word();
test_bad_frames_rejected();
test_find_reply_skips_echo_and_noise();
test_field_range_checked();
disp('protocol_test: all checks passed');
end

%> @brief Captured frames, as hex strings.
%> @return Struct of frames named as in the Python and C++ tests.
function frames = captured()
frames.CMD_ZERO_TORQUE = 'feee90800000000000000000000000009b1e4106';
frames.CMD_DAMPING = 'feee9080000000000000000000001900541ed614';
frames.CMD_IDLE = 'feee000000000000000000000000000080462079';
frames.CMD_HOLD = 'feee908000000000c8610000c2010a0071dc7c64';
frames.CMD_RESTART = 'feee7000000000000000000000000000378838d7';
frames.REPLY_ROBOT = 'fcee12242361fffff8ffc63dfdff00000000cc0b00c82563ad8c';
frames.REPLY_BENCH = 'fcee041e1c30fbfff9ff1b08010000000000c00b000057221ecf';
frames.REPLY_FAULT = 'fcee042626300000fdffc15d010008000000a80b0000a190371a';
end

%> @brief Converts a hex string to a uint8 row.
%> @param text Hex digits.
%> @return The bytes.
function bytes = hex(text)
bytes = uint8(hex2dec(reshape(text, 2, []).')).';
end

%> @brief A run-mode command for id 0 as the G1 controller sends it.
%> @return The command.
function cmd = run_cmd()
cmd = g1motor.MotorCmd('id', 0, 'mode', g1motor.MODE_RUN, 'flag', true, ...
                       'byte3', 128);
end

%> @brief Checksums of a captured command and reply.
function test_checksum()
f = captured();
command = hex(f.CMD_ZERO_TORQUE);
assert(g1motor.crc32_core(command(1:16)) == hex2dec('06411E9B'));
reply = hex(f.REPLY_ROBOT);
assert(g1motor.crc32_core(reply(3:22)) == hex2dec('8CAD6325'));
end

%> @brief Encoded commands equal the captured frames.
function test_encode_matches_capture()
f = captured();
assert(isequal(g1motor.MotorCmd('id', 0).encode(), hex(f.CMD_IDLE)));
assert(isequal(run_cmd().encode(), hex(f.CMD_ZERO_TORQUE)));
damping = run_cmd();
damping.kd = 25;
assert(isequal(damping.encode(), hex(f.CMD_DAMPING)));
hold = run_cmd();
hold.pos = 25032;
hold.kp = 450;
hold.kd = 10;
assert(isequal(hold.encode(), hex(f.CMD_HOLD)));
restart = g1motor.MotorCmd('id', 0, 'mode', g1motor.MODE_RESTART);
assert(isequal(restart.encode(), hex(f.CMD_RESTART)));
end

%> @brief A command survives encode and decode.
function test_command_round_trip()
cmd = g1motor.MotorCmd('id', 6, 'mode', g1motor.MODE_RUN, 'flag', true, ...
    'byte3', 144, 'tau', -12, 'vel', 345, 'pos', -123456, 'kp', 300, ...
    'kd', 80);
back = g1motor.MotorCmd.decode(cmd.encode());
assert(isequal(back.to_row(), cmd.to_row()));
end

%> @brief Fields of captured robot and bench replies.
function test_decode_reply()
f = captured();
state = g1motor.MotorState.decode(hex(f.REPLY_ROBOT));
assert(state.id == 2 && state.mode == 1 && state.pos == -180794);
assert(state.vel == -8 && state.tau == -1);
assert(state.temp_coil == 36 && state.temp_shell == 35);
assert(state.voltage() == 48.5 && ~state.fault());
bench = g1motor.MotorState.decode(hex(f.REPLY_BENCH));
assert(bench.id == 4 && bench.mode == g1motor.MODE_IDLE);
assert(bench.pos == 67611 && bench.voltage() == 24.0);
end

%> @brief A captured reply with status 0x8 reports a fault.
function test_fault_word()
f = captured();
state = g1motor.MotorState.decode(hex(f.REPLY_FAULT));
assert(state.fault() && state.error == 8);
assert(state.mode == g1motor.MODE_IDLE);
end

%> @brief Corrupt or short frames decode to [].
function test_bad_frames_rejected()
f = captured();
reply = hex(f.REPLY_ROBOT);
broken = reply;
broken(11) = bitxor(broken(11), 1);
assert(isempty(g1motor.MotorState.decode(broken)));
assert(isempty(g1motor.MotorState.decode(reply(1:end - 1))));
command = hex(f.CMD_IDLE);
command(end) = 0;
assert(isempty(g1motor.MotorCmd.decode(command)));
end

%> @brief A reply is found after echo and noise.
function test_find_reply_skips_echo_and_noise()
f = captured();
buffer = [hex(f.CMD_IDLE), hex('fcee00'), hex(f.REPLY_BENCH)];
found = g1motor.find_reply(buffer);
assert(found.id == 4);
assert(isempty(g1motor.find_reply(hex(f.CMD_IDLE))));
end

%> @brief Out-of-range fields and ids raise an error.
function test_field_range_checked()
assert(raises(@() g1motor.MotorCmd('id', 4, 'kp', 40000).encode()));
assert(raises(@() g1motor.MotorCmd('id', 15).encode()));
end

%> @brief Whether calling a function raises an error.
%> @param fn Function handle to call.
%> @return True if it raised.
function threw = raises(fn)
threw = false;
try
    fn();
catch
    threw = true;
end
end

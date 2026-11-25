%> @file protocol_test.m
%> @brief Frame tests against frames captured from a G1 arm bus. No hardware.
%>
%> Errors on the first failed check.

%> @brief Runs the frame checks through the MEX encode and decode commands.
function protocol_test()
addpath(fullfile(fileparts(mfilename('fullpath')), '..'));

idle = g1motor_mex('encode', struct('id', 0));
assert(isequal(idle, hex('feee000000000000000000000000000080462079')));
run = struct('id', 0, 'mode', 1, 'flag', 1, 'byte3', 128);
assert(isequal(g1motor_mex('encode', run), ...
               hex('feee90800000000000000000000000009b1e4106')));
hold = run;  hold.pos = 25032;  hold.kp = 450;  hold.kd = 10;
assert(isequal(g1motor_mex('encode', hold), ...
               hex('feee908000000000c8610000c2010a0071dc7c64')));
restart = struct('id', 0, 'mode', 7);
assert(isequal(g1motor_mex('encode', restart), ...
               hex('feee7000000000000000000000000000378838d7')));

reply = hex('fcee12242361fffff8ffc63dfdff00000000cc0b00c82563ad8c');
state = g1motor_mex('decode', reply);
assert(state.id == 2 && state.mode == 1 && state.pos == -180794);
assert(state.vel == -8 && state.tau == -1 && state.voltage == 48.5);
assert(state.temp_coil == 36 && state.temp_shell == 35 && ~state.fault);

fault = g1motor_mex('decode', ...
    hex('fcee042626300000fdffc15d010008000000a80b0000a190371a'));
assert(fault.fault && fault.error == 8 && fault.voltage == 24);

broken = reply;
broken(11) = bitxor(broken(11), 1);
assert(isempty(g1motor_mex('decode', broken)));

threw = false;
try
    g1motor_mex('encode', struct('id', 4, 'kp', 40000));
catch
    threw = true;
end
assert(threw);
disp('protocol_test: all checks passed');
end

%> @brief Converts a hex string to a uint8 row.
function bytes = hex(text)
bytes = uint8(hex2dec(reshape(text, 2, []).')).';
end

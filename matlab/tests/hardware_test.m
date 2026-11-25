%> @file hardware_test.m
%> @brief Live test on motors with no controller on the bus.
%>
%> Scans, enables every motor found, holds position for 2 s, restarts one
%> motor and checks it comes back. The joints are held in place, not moved.
%> Errors on the first failed check.

%> @brief Runs the live test.
%> @param port Device name; omit for the only FTDI adapter.
function hardware_test(port)
if nargin < 1, port = ''; end
addpath(fullfile(fileparts(mfilename('fullpath')), '..'));
bus = g1motor.MotorBus(port);
cleanup = onCleanup(@() bus.close());

found = bus.scan();
assert(found.Count > 0, 'no motor answered');
ids = cell2mat(keys(found));
any_fault = false;
for id = ids
    state = found(id);
    assert(state.mode == g1motor.MODE_IDLE, 'idle reply reports mode 0');
    assert(state.voltage() > 10 && state.voltage() < 60);
    assert(state.temp_coil >= state.temp_shell);
    any_fault = any_fault || state.fault();
    fprintf('id %d: pos %d, %.1f V, status 0x%x\n', id, state.pos, ...
            state.voltage(), state.error);
end
if any_fault, bus.restart(ids); end

absent = mod(ids(1) + 7, 15);
if ~isKey(found, absent)
    assert(isempty(bus.transfer(g1motor.MotorCmd('id', absent))), ...
           'an absent id stays silent');
end

start = bus.enable(ids);
target = zeros(size(ids));
for k = 1:numel(ids)
    state = start(ids(k));
    assert(state.mode == g1motor.MODE_RUN, 'enable reaches run mode');
    target(k) = state.pos;
end

worst_error = 0;
cycles = 0;
timer = tic;
while toc(timer) < 2
    for k = 1:numel(ids)
        kd = 80 - 40 * (ids(k) == 4);
        state = bus.command(ids(k), target(k), 0, 0, 300, kd);
        assert(~isempty(state), 'command gets a reply');
        worst_error = max(worst_error, abs(state.pos - target(k)));
    end
    cycles = cycles + 1;
end
fprintf('hold: largest error %d counts, %.0f exchanges/s per motor\n', ...
        worst_error, cycles / 2);
assert(worst_error < 600, 'position hold stays on target');
assert(cycles / 2 > 100, 'polling rate above 100 per second');

bus.disable();
idle = bus.read(ids(1));
assert(idle.mode == g1motor.MODE_IDLE, 'disable returns to idle');

after = bus.restart(ids(1));
assert(isKey(after, ids(1)), 'motor answers after restart');
state = after(ids(1));
assert(state.mode == g1motor.MODE_IDLE && ~state.fault());
disp('hardware_test: all checks passed');
end

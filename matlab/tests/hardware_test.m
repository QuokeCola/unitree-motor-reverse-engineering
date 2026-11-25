%> @file hardware_test.m
%> @brief Live test on motors with no controller on the bus.
%>
%> Scans, enables every motor found, holds position for 2 s, restarts one
%> motor and checks it comes back. The joints are held in place, not moved.
%> Errors on the first failed check.

%> @brief Runs the live test.
%> @param port Device name; omit for the only FTDI adapter (Linux).
function hardware_test(port)
if nargin < 1, port = ''; end
addpath(fullfile(fileparts(mfilename('fullpath')), '..'));
bus = G1MotorBus(port);
cleanup = onCleanup(@() delete(bus));

found = bus.scan();
assert(~isempty(found), 'no motor answered');
ids = [found.id];
assert(all([found.mode] == 0), 'idle reply should report mode 0');
assert(all([found.voltage] > 10 & [found.voltage] < 60));
assert(all([found.temp_coil] >= [found.temp_shell]));
disp(struct2table(found));
if any([found.fault]), bus.restart(ids); end

start = bus.enable(ids);
assert(all([start.mode] == 1), 'enable should reach run mode');

target = [start.pos];
worst = 0;
cycles = 0;
timer = tic;
while toc(timer) < 2
    cycles = cycles + 1;
    for k = 1:numel(ids)
        kd = 80 - 40 * (ids(k) == 4);
        state = bus.commandRaw(ids(k), target(k), 0, 0, 300, kd);
        assert(~isempty(state), 'command got no reply');
        worst = max(worst, abs(state.pos - target(k)));
    end
end
fprintf('hold: largest error %d counts, %.0f exchanges/s per motor\n', ...
        worst, cycles / 2);
assert(worst < 600, 'position hold drifted');
assert(cycles / 2 > 100, 'polling rate too low');

bus.disable();
idle = bus.read(ids(1));
assert(idle.mode == 0, 'disable should return to idle');

after = bus.restart(ids(1));
assert(~isempty(after) && after.mode == 0 && ~after.fault);
disp('hardware_test: all checks passed');
end

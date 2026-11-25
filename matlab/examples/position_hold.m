%> @file position_hold.m
%> @brief Holds every joint where it is with a position loop.
%>
%> Usage: position_hold, position_hold(seconds), position_hold(seconds, port)
%> Gains are those tuned on a bench wrist (ids 4, 5, 6) at 24 V.

%> @brief Holds every motor found at its start position and prints the error.
%> @param seconds How long to hold; default 3.
%> @param port Device name; omit for the only FTDI adapter.
function position_hold(seconds, port)
if nargin < 1, seconds = 3; end
if nargin < 2, port = ''; end
addpath(fullfile(fileparts(mfilename('fullpath')), '..'));
bus = g1motor.MotorBus(port);
cleanup = onCleanup(@() bus.close());
ids = cell2mat(keys(bus.scan()));
start = bus.enable(ids);
target = zeros(size(ids));
for k = 1:numel(ids)
    state = start(ids(k));
    target(k) = state.pos;
end
worst = zeros(size(ids));
timer = tic;
while toc(timer) < seconds
    for k = 1:numel(ids)
        kd = 80 - 40 * (ids(k) == 4);
        state = bus.command(ids(k), target(k), 0, 0, 300, kd);
        if isempty(state), continue; end
        worst(k) = max(worst(k), abs(state.pos - target(k)));
    end
end
for k = 1:numel(ids)
    fprintf('id %d: held %d counts, largest error %d counts\n', ids(k), ...
            target(k), worst(k));
end
end  % Closing the bus returns the motors to idle.

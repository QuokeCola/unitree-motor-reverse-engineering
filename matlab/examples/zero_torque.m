%> @file zero_torque.m
%> @brief Frees the joints and prints how far each was moved.
%>
%> Usage: zero_torque, zero_torque(seconds) or zero_torque(seconds, port)

%> @brief Enables every motor found with zero torque and records its travel.
%> @param seconds How long to keep the joints free; default 10.
%> @param port Device name; omit for the only FTDI adapter.
function zero_torque(seconds, port)
if nargin < 1, seconds = 10; end
if nargin < 2, port = ''; end
addpath(fullfile(fileparts(mfilename('fullpath')), '..'));
bus = g1motor.MotorBus(port);
cleanup = onCleanup(@() bus.close());
ids = cell2mat(keys(bus.scan()));
start = bus.enable(ids);  % run mode, zero torque
low = zeros(size(ids));
for k = 1:numel(ids)
    state = start(ids(k));
    low(k) = state.pos;
end
high = low;
timer = tic;
while toc(timer) < seconds
    for k = 1:numel(ids)
        state = bus.command(ids(k));  % all zero = zero torque
        if isempty(state), continue; end
        low(k) = min(low(k), state.pos);
        high(k) = max(high(k), state.pos);
    end
end
for k = 1:numel(ids)
    fprintf('id %d: moved between %d and %d counts\n', ids(k), low(k), ...
            high(k));
end
end

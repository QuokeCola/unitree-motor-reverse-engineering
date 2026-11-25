%> @file sine.m
%> @brief Slow sine on the joints, using stored end stops.
%>
%> Usage: sine(limits_file), sine(limits_file, period, seconds, port)
%> Record the end stops first, after every power-on, with
%> `g1motor limits --ids 4,5,6 --out limits.json`. Uses 80 % of each joint's
%> travel. Gains are those tuned on a bench wrist (ids 4, 5, 6) at 24 V.

%> @brief Moves every joint in the limits file through a sine.
%> @param limits_file JSON file of end stops.
%> @param period Sine period in seconds; default 8.
%> @param seconds Duration of the sine; default three periods.
%> @param port Device name; omit for the only FTDI adapter.
function sine(limits_file, period, seconds, port)
if nargin < 2, period = 8; end
if nargin < 3, seconds = 3 * period; end
if nargin < 4, port = ''; end
addpath(fullfile(fileparts(mfilename('fullpath')), '..'));
KP = 300;          % position gain
MAX_LAG = 30000;   % largest allowed gap between target and position

joints = g1motor.load_joints(limits_file);
ids = cell2mat(keys(joints));
bus = g1motor.MotorBus(port);
cleanup = onCleanup(@() bus.close());
for id = ids
    bus.set_joint(id, joints(id));
end
start = bus.enable(ids);  % also re-references each joint
mid = zeros(size(ids));
amplitude = zeros(size(ids));
target = zeros(size(ids));
kd = 80 - 40 * (ids == 4);
for k = 1:numel(ids)
    joint = bus.joint(ids(k));
    state = start(ids(k));
    mid(k) = (joint.low + joint.high) / 2;
    amplitude(k) = 0.4 * (joint.high - joint.low);
    target(k) = state.pos;
    fprintf('id %d: travel %.0f deg, now at %+.1f deg\n', ids(k), ...
            rad2deg(joint.travel()), rad2deg(joint.angle(state.pos)));
end

% Move to mid-travel at 50,000 counts/s.
timer = tic;
last = 0;
moving = true;
while moving
    now = toc(timer);
    dt = now - last;
    last = now;
    moving = false;
    for k = 1:numel(ids)
        gap = mid(k) - target(k);
        step = min(50000 * dt, abs(gap));
        target(k) = target(k) + sign(gap) * step;
        if abs(mid(k) - target(k)) > 1, moving = true; end
        bus.command(ids(k), round(target(k)), 0, 0, KP, kd(k));
    end
end

omega = 2 * pi / period;
timer = tic;
t = 0;
ok = true;
while ok && t < seconds
    % The amplitude fades in and out over one period.
    fade = min([t, seconds - t, period]) / period;
    envelope = 0.5 - 0.5 * cos(pi * fade);
    for k = 1:numel(ids)
        goal = mid(k) + envelope * amplitude(k) * sin(omega * t);
        rate = envelope * amplitude(k) * omega * cos(omega * t);
        state = bus.command(ids(k), round(goal), ...
            g1motor.units.pos_rate_to_vel(rate), 0, KP, kd(k));
        if ~isempty(state) && abs(state.pos - goal) > MAX_LAG
            fprintf('id %d: lag %.0f counts, stopping\n', ids(k), ...
                    abs(state.pos - goal));
            ok = false;
        end
    end
    t = toc(timer);
end
end  % Closing the bus returns the motors to idle.

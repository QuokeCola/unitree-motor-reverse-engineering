%> @file example_hold.m
%> @brief Holds the motors where they are for 3 s and plots the error.
%>
%> Gains are those tuned on a bench wrist (ids 4, 5, 6) at 24 V.
addpath(fullfile(fileparts(mfilename('fullpath')), '..'));
bus = G1MotorBus();
found = bus.scan();
disp(struct2table(found));
ids = [found.id];
kd = 80 * ones(size(ids));
kd(ids == 4) = 40;

start = bus.enable(ids);
target = [start.pos];
count = 0;
errors = zeros(2000, numel(ids));
times = zeros(2000, 1);
timer = tic;
while toc(timer) < 3
    count = count + 1;
    for k = 1:numel(ids)
        state = bus.commandRaw(ids(k), target(k), 0, 0, 300, kd(k));
        if ~isempty(state), errors(count, k) = state.pos - target(k); end
    end
    times(count) = toc(timer);
end
bus.disable();
delete(bus);

fprintf('%.0f exchanges per second per motor\n', count / times(count));
plot(times(1:count), errors(1:count, :));
xlabel('time (s)');
ylabel('position error (counts)');
legend(compose('id %d', ids));

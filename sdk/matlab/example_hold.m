% Hold the bench wrist joints where they are for 3 s and plot the position error.
% Gains are the ones tuned on that wrist (ids 4, 5, 6) at 24 V; start lower on other joints.
bus = G1MotorBus();
found = bus.scan();
disp(struct2table(found));
ids = [found.id];
kd = 80 * ones(size(ids));  kd(ids == 4) = 40;

start = bus.enable(ids);
target = [start.pos];
n = 0;  log = zeros(2000, numel(ids));  t = zeros(2000, 1);
t0 = tic;
while toc(t0) < 3
    n = n + 1;
    for k = 1:numel(ids)
        s = bus.commandRaw(ids(k), target(k), 0, 0, 300, kd(k));
        if ~isempty(s), log(n, k) = s.pos - target(k); end
    end
    t(n) = toc(t0);
end
bus.disable();
delete(bus);

fprintf('%.0f exchanges per second per motor\n', n / t(n));
plot(t(1:n), log(1:n, :));  xlabel('time (s)');  ylabel('position error (counts)');
legend(compose('id %d', ids));

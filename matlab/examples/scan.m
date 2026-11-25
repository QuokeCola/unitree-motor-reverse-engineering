%> @file scan.m
%> @brief Lists the motors on the bus. Usage: scan or scan(port)

%> @brief Scans the bus and prints one line per motor.
%> @param port Device name; omit for the only FTDI adapter.
function scan(port)
if nargin < 1, port = ''; end
addpath(fullfile(fileparts(mfilename('fullpath')), '..'));
bus = g1motor.MotorBus(port);
cleanup = onCleanup(@() bus.close());
found = bus.scan();
if found.Count == 0, disp('no motors answered on ids 0-14'); end
for id = cell2mat(keys(found))
    state = found(id);
    line = sprintf(['id %d: mode %d, position %d counts, %.1f V, ' ...
                    'coil %d C, shell %d C'], id, state.mode, state.pos, ...
                   state.voltage(), state.temp_coil, state.temp_shell);
    if state.fault(), line = [line ', FAULT']; end %#ok<AGROW>
    disp(line);
end
end

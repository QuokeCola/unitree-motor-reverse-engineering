# MATLAB interface

`G1MotorBus` class over a MEX build of the C++ library in `../cpp`. MATLAB R2018a or later and
a C++17 compiler set up for MEX (`mex -setup C++`).

```matlab
build_mex                              % once, and after the C++ sources change
bus = G1MotorBus();                    % or G1MotorBus('COM5')
start = bus.enable([4 5 6]);           % run mode, zero torque
state = bus.command(4, 'pos', start(1).pos, 'kp', 300, 'kd', 40);   % call in a loop
bus.disable();                         % idle brakes the joints
delete(bus)
```

Methods: `scan`, `enable`, `disable`, `restart`, `command`, `commandRaw` (positional arguments,
faster in a loop), `read`, `setJoint`, `getJoint`, `jointAngle`, `listen`. States are structs with
`id`, `mode`, `pos`, `vel`, `tau`, `temp_coil`, `temp_shell`, `voltage`, `error`, `fault`. Only one
bus can be open per MATLAB session.

## Tests and example

```matlab
addpath tests
protocol_test          % frame encode and decode through the MEX file, no hardware
hardware_test          % motors on a bus without a controller
```

Example: `examples/example_hold.m`.

`build_mex` accepts a file that `mex` links but then rejects in its final check, provided the
file loads and encodes a frame; this happens with very new compilers.

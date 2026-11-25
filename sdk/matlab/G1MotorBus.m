classdef G1MotorBus < handle
    %G1MOTORBUS Unitree G1 joint motors on an RS-485 chain without a controller (unofficial).
    %
    %   bus = G1MotorBus();                % the only FTDI adapter (Linux), or G1MotorBus('COM5')
    %   s = bus.scan();                    % struct array: id, mode, pos, vel, tau, temp_coil, ...
    %   start = bus.enable([4 5 6]);       % run mode, zero torque: joints are free
    %   s = bus.command(4, 'pos', start(1).pos, 'kp', 300, 'kd', 40);   % call in a steady loop
    %   bus.disable();                     % back to idle, which brakes the joints
    %   delete(bus)                        % also returns the motors to idle
    %
    %   All values are raw counts. torque = tau + kp*(pos - measured) + kd*(vel - measured).
    %   Position: 32768 counts per rotor turn. Velocity: 1 count = 128 position counts per second.
    %   A motor that is no longer polled drops to idle after 0.5-1 s.
    %   Only one bus can be open per MATLAB session. Build the MEX file first with build_mex.

    methods
        function obj = G1MotorBus(port)
            if nargin < 1, port = ''; end
            g1motor_mex('open', char(port));
        end

        function delete(~)
            g1motor_mex('close');
        end

        function s = scan(~)
            %SCAN Poll ids 0-14 with the idle frame; struct array of the motors that answer.
            s = g1motor_mex('scan');
        end

        function s = enable(~, ids)
            %ENABLE Run mode with zero torque. Errors if a motor reports a fault (see RESTART).
            s = g1motor_mex('enable', double(ids));
        end

        function disable(~, ids)
            %DISABLE Back to idle, which brakes the joint. Also the way to stop a moving joint.
            if nargin < 2, ids = []; end
            g1motor_mex('disable', double(ids));
        end

        function s = restart(~, ids)
            %RESTART Mode 7: clears a latched fault, about 0.3 s. Positions may come back shifted
            %   by whole output turns; joints set with SETJOINT are re-referenced.
            s = g1motor_mex('restart', double(ids));
        end

        function s = command(~, id, varargin)
            %COMMAND One hybrid command to an enabled motor; returns its state (empty if no reply).
            %   s = bus.command(id, 'pos', p, 'vel', v, 'tau', t, 'kp', kp, 'kd', kd)
            p = inputParser;
            for name = {'pos', 'vel', 'tau', 'kp', 'kd'}
                addParameter(p, name{1}, 0, @(x) isnumeric(x) && isscalar(x));
            end
            parse(p, varargin{:});
            r = p.Results;
            s = g1motor_mex('command', id, r.pos, r.vel, r.tau, r.kp, r.kd);
        end

        function s = commandRaw(~, id, pos, vel, tau, kp, kd)
            %COMMANDRAW Same as COMMAND with positional arguments; faster inside a control loop.
            s = g1motor_mex('command', id, pos, vel, tau, kp, kd);
        end

        function s = read(~, id)
            %READ Fresh state. On an enabled motor this sends zero torque, which drops a hold.
            s = g1motor_mex('read', id);
        end

        function setJoint(~, id, gearRatio, low, high)
            %SETJOINT Gear ratio and end stops in raw counts. COMMAND then clamps position targets,
            %   and ENABLE / RESTART shift the end stops by whole output turns to match the reading.
            g1motor_mex('set_joint', id, gearRatio, low, high);
        end

        function j = getJoint(~, id)
            %GETJOINT Struct with gear_ratio, low, high, zero, turn_counts (after re-referencing).
            j = g1motor_mex('get_joint', id);
        end

        function a = jointAngle(obj, id, pos)
            %JOINTANGLE Output-shaft angle in rad, measured from the middle of the joint's travel.
            j = obj.getJoint(id);
            a = (pos - j.zero) / j.turn_counts * 2 * pi;
        end

        function bytes = listen(~, seconds)
            %LISTEN Raw bytes from the bus for this many seconds; nothing is transmitted.
            bytes = g1motor_mex('listen', seconds);
        end
    end
end

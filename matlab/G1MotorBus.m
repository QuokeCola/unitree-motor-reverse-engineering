%> @file G1MotorBus.m
%> @brief MATLAB interface to Unitree G1 joint motors (unofficial).

%> @brief One RS-485 chain of motors, with MATLAB as the only master.
%>
%> @code
%>   bus = G1MotorBus();              % or G1MotorBus('COM5')
%>   start = bus.enable([4 5 6]);     % run mode, joints free
%>   s = bus.command(4, 'pos', start(1).pos, 'kp', 300, 'kd', 40);
%>   bus.disable();                   % idle brakes the joints
%> @endcode
%>
%> All values are raw counts. The motor applies
%> tau + kp*(pos - measured) + kd*(vel - measured). A motor that is no
%> longer polled drops to idle after 0.5-1 s. Only one bus can be open per
%> MATLAB session. Build the MEX file first with build_mex.
classdef G1MotorBus < handle
    methods
        %> @brief Opens the adapter at 6 Mbaud.
        %> @param port Device name; omit for the only FTDI adapter (Linux).
        function obj = G1MotorBus(port)
            if nargin < 1, port = ''; end
            g1motor_mex('open', char(port));
        end

        %> @brief Returns enabled motors to idle and closes the port.
        function delete(~)
            g1motor_mex('close');
        end

        %> @brief Polls ids 0-14 with the idle frame.
        %> @return Struct array of the motors that answered: id, mode, pos,
        %>         vel, tau, temp_coil, temp_shell, voltage, error, fault.
        function states = scan(~)
            states = g1motor_mex('scan');
        end

        %> @brief Puts motors in run mode with zero torque; joints are free.
        %> @param ids Motor ids.
        %> @return Struct array with the state of each motor.
        function states = enable(~, ids)
            states = g1motor_mex('enable', double(ids));
        end

        %> @brief Returns motors to idle, which brakes the joint passively.
        %>
        %> This is also the way to stop a moving joint.
        %> @param ids Motor ids; omit for every enabled motor.
        function disable(~, ids)
            if nargin < 2, ids = []; end
            g1motor_mex('disable', double(ids));
        end

        %> @brief Restarts motors (mode 7), clearing a latched fault.
        %>
        %> The position reading may come back shifted by whole output
        %> turns; joints set with setJoint are re-referenced.
        %> @param ids Motor ids.
        %> @return Struct array with the state of each motor afterwards.
        function states = restart(~, ids)
            states = g1motor_mex('restart', double(ids));
        end

        %> @brief Sends one hybrid command to an enabled motor.
        %> @param id Motor id.
        %> @param varargin Name-value pairs: 'pos', 'vel', 'tau', 'kp', 'kd'.
        %> @return State struct; empty if the motor did not reply.
        function state = command(~, id, varargin)
            parser = inputParser;
            for name = {'pos', 'vel', 'tau', 'kp', 'kd'}
                addParameter(parser, name{1}, 0, ...
                             @(x) isnumeric(x) && isscalar(x));
            end
            parse(parser, varargin{:});
            r = parser.Results;
            state = g1motor_mex('command', id, r.pos, r.vel, r.tau, ...
                                r.kp, r.kd);
        end

        %> @brief Same as command with positional arguments; faster in a loop.
        %> @param id Motor id.
        %> @param pos Position target.
        %> @param vel Velocity target.
        %> @param tau Feed-forward torque.
        %> @param kp Position gain.
        %> @param kd Damping gain.
        %> @return State struct; empty if the motor did not reply.
        function state = commandRaw(~, id, pos, vel, tau, kp, kd)
            state = g1motor_mex('command', id, pos, vel, tau, kp, kd);
        end

        %> @brief Reads a motor's state.
        %>
        %> On an enabled motor this sends zero torque, which drops a hold.
        %> @param id Motor id.
        %> @return State struct; empty if the motor did not reply.
        function state = read(~, id)
            state = g1motor_mex('read', id);
        end

        %> @brief Attaches gear ratio and end stops to a motor.
        %>
        %> command then limits position targets, and enable / restart shift
        %> the end stops by whole output turns to match the reading.
        %> @param id Motor id.
        %> @param gearRatio Rotor turns per output turn.
        %> @param low Lower end stop in counts.
        %> @param high Upper end stop in counts.
        function setJoint(~, id, gearRatio, low, high)
            g1motor_mex('set_joint', id, gearRatio, low, high);
        end

        %> @brief Reads back a joint after re-referencing.
        %> @param id Motor id.
        %> @return Struct: gear_ratio, low, high, zero, turn_counts.
        function joint = getJoint(~, id)
            joint = g1motor_mex('get_joint', id);
        end

        %> @brief Converts a position to an output-shaft angle.
        %> @param id Motor id of a joint set with setJoint.
        %> @param pos Position in counts.
        %> @return Angle in rad from the middle of the joint's travel.
        function angle = jointAngle(obj, id, pos)
            joint = obj.getJoint(id);
            angle = (pos - joint.zero) / joint.turn_counts * 2 * pi;
        end

        %> @brief Receives without transmitting.
        %> @param seconds How long to listen.
        %> @return Raw bytes as a uint8 row.
        function bytes = listen(~, seconds)
            bytes = g1motor_mex('listen', seconds);
        end
    end
end

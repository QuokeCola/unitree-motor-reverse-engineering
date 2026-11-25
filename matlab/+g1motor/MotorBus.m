%> @file MotorBus.m
%> @brief MotorBus: drive G1 joint motors on a bus without a controller.

%> @brief One RS-485 chain of motors, with MATLAB as the only master.
%>
%> @code
%>   bus = g1motor.MotorBus();
%>   start = bus.enable([4 5 6]);        % run mode, joints free
%>   start4 = start(4);
%>   bus.command(4, start4.pos, 0, 0, 300, 40);   % in a steady loop
%>   bus.close();                        % idle brakes the joints
%> @endcode
%>
%> Motors answer only when polled, so every command returns that motor's
%> fresh state. A motor that is no longer polled drops to idle after
%> 0.5-1 s. Only one bus can be open per MATLAB session.
classdef MotorBus < handle
    properties (Access = private)
        %> Joints attached, by id.
        joints
        %> True while the port is open.
        is_open = false
    end

    methods
        %> @brief Opens the adapter at 6 Mbaud.
        %> @param port Device name; '' selects the only FTDI adapter.
        %> @param reply_timeout Longest wait for a reply, in seconds.
        %> @param check_silent Listen first and refuse a bus with traffic.
        function bus = MotorBus(port, reply_timeout, check_silent)
            if nargin < 1, port = ''; end
            if nargin < 2, reply_timeout = 0.004; end
            if nargin < 3, check_silent = true; end
            bus.joints = containers.Map('KeyType', 'double', ...
                                        'ValueType', 'any');
            g1motor_mex('open', char(port), reply_timeout, ...
                        double(check_silent));
            bus.is_open = true;
        end

        %> @brief Closes the bus when the object is cleared.
        function delete(bus)
            bus.close();
        end

        %> @brief Returns enabled motors to idle and closes the port.
        function close(bus)
            if bus.is_open
                bus.is_open = false;
                g1motor_mex('close');
            end
        end

        %> @brief Receives without transmitting.
        %> @param seconds How long to listen.
        %> @return The raw bytes received, as a uint8 row.
        function bytes = listen(~, seconds)
            bytes = g1motor_mex('listen', seconds);
        end

        %> @brief Sends one frame and waits for that motor's reply.
        %> @param cmd MotorCmd to send.
        %> @return The reply, or [] on timeout or a bad checksum.
        function state = transfer(~, cmd)
            state = g1motor.MotorState.from_rows( ...
                g1motor_mex('transfer', cmd.to_row()));
        end

        %> @brief Polls ids 0-14 with the idle frame.
        %> @param tries Attempts per id; default 3.
        %> @return containers.Map of the motors that answered, by id.
        function found = scan(~, tries)
            if nargin < 2, tries = 3; end
            found = g1motor.MotorState.to_map(g1motor_mex('scan', tries));
        end

        %> @brief Reads a motor's state.
        %>
        %> On an enabled motor this sends a zero-torque command, which drops
        %> a position hold.
        %> @param id Motor to read.
        %> @return The state, or [] if it did not reply.
        function state = read(~, id)
            state = g1motor.MotorState.from_rows(g1motor_mex('read', id));
        end

        %> @brief Puts motors in run mode with zero torque; joints are free.
        %>
        %> Joints attached with set_joint() are re-referenced to the reading.
        %> @param ids Motors to enable.
        %> @return containers.Map with the state of each motor, by id.
        function states = enable(bus, ids)
            states = g1motor.MotorState.to_map( ...
                g1motor_mex('enable', double(ids)));
            try
                bus.sync_joints(states);
            catch sync_error
                bus.disable(ids);
                error('g1motor:MotorFaultError', '%s', sync_error.message);
            end
        end

        %> @brief Returns motors to idle, which brakes the joint passively.
        %>
        %> This is also the way to stop a moving joint.
        %> @param ids Motors to idle; omit for every enabled motor.
        function disable(~, ids)
            if nargin < 2, ids = []; end
            g1motor_mex('disable', double(ids));
        end

        %> @brief Restarts motors (mode 7), clearing a latched fault.
        %>
        %> The position reading may come back shifted by whole output turns;
        %> attached joints are re-referenced.
        %> @param ids Motors to restart.
        %> @param wait Seconds allowed for the restart; default 0.5.
        %> @return containers.Map with the state of each motor afterwards.
        function states = restart(bus, ids, wait)
            if nargin < 3, wait = 0.5; end
            states = g1motor.MotorState.to_map( ...
                g1motor_mex('restart', double(ids), wait));
            try
                bus.sync_joints(states);
            catch sync_error
                error('g1motor:MotorFaultError', '%s', sync_error.message);
            end
        end

        %> @brief Sends one hybrid command, in raw counts, to an enabled
        %>        motor.
        %>
        %> Position targets are limited to the end stops of an attached
        %> joint.
        %> @param id Motor to command.
        %> @param pos Position target; default 0.
        %> @param vel Velocity target; default 0.
        %> @param tau Feed-forward torque; default 0.
        %> @param kp Position gain; default 0.
        %> @param kd Damping gain; default 0.
        %> @return The motor's state, or [] if it did not reply.
        function state = command(bus, id, pos, vel, tau, kp, kd)
            if nargin < 3, pos = 0; end
            if nargin < 4, vel = 0; end
            if nargin < 5, tau = 0; end
            if nargin < 6, kp = 0; end
            if nargin < 7, kd = 0; end
            if kp ~= 0 && isKey(bus.joints, id)
                attached = bus.joints(id);
                pos = attached.clamp(pos);
            end
            state = g1motor.MotorState.from_rows( ...
                g1motor_mex('command', id, pos, vel, tau, kp, kd));
        end

        %> @brief Attaches a joint description to a motor.
        %> @param id Motor the joint belongs to.
        %> @param joint Joint with gear ratio, end stops and zero.
        function set_joint(bus, id, joint)
            bus.joints(id) = joint;
        end

        %> @brief The joint attached to a motor.
        %> @param id Motor id.
        %> @return The Joint, as re-referenced by enable() and restart().
        function attached = joint(bus, id)
            attached = bus.joints(id);
        end

        %> @brief Whether a motor was enabled through this object.
        %> @param id Motor id.
        %> @return True if it is enabled.
        function enabled = is_enabled(~, id)
            enabled = g1motor_mex('is_enabled', id);
        end
    end

    methods (Access = private)
        %> @brief Re-references attached joints to fresh position readings.
        %> @param states containers.Map from id to MotorState.
        function sync_joints(bus, states)
            ids = cell2mat(keys(states));
            for id = ids
                if isKey(bus.joints, id)
                    attached = bus.joints(id);
                    state = states(id);
                    attached.sync(state.pos);
                end
            end
        end
    end
end

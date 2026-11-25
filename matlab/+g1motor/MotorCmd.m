%> @file MotorCmd.m
%> @brief Command frame.

%> @brief One command frame (controller to motor).
%>
%> The motor applies tau + kp * (pos - measured) + kd * (vel - measured).
classdef MotorCmd
    properties
        %> Motor id, 0-14.
        id = 0
        %> MODE_IDLE, MODE_RUN or MODE_RESTART.
        mode = 0
        %> Feed-forward torque, int16.
        tau = 0
        %> Velocity target, int16.
        vel = 0
        %> Position target, int32.
        pos = 0
        %> Position gain, int16.
        kp = 0
        %> Damping gain, int16.
        kd = 0
        %> Top bit of the id/mode byte; not required.
        flag = false
        %> Sequence byte; not required.
        byte3 = 0
    end

    methods
        %> @brief Creates a command from name-value pairs.
        %> @param varargin Pairs such as 'id', 4, 'mode', 1 (or id=4, mode=1).
        function cmd = MotorCmd(varargin)
            for k = 1:2:numel(varargin)
                cmd.(char(varargin{k})) = varargin{k + 1};
            end
        end

        %> @brief Builds the 20-byte frame, checksum included.
        %> @return The frame as a uint8 row.
        function frame = encode(cmd)
            frame = g1motor_mex('encode', cmd.to_row());
        end

        %> @brief Packs the fields for the MEX gateway.
        %> @return Row: id mode tau vel pos kp kd flag byte3.
        function row = to_row(cmd)
            row = double([cmd.id, cmd.mode, cmd.tau, cmd.vel, cmd.pos, ...
                          cmd.kp, cmd.kd, cmd.flag, cmd.byte3]);
        end
    end

    methods (Static)
        %> @brief Parses a command frame.
        %> @param frame Exactly 20 bytes (uint8).
        %> @return The command, or [] if the header or checksum is wrong.
        function cmd = decode(frame)
            row = g1motor_mex('decode_cmd', uint8(frame));
            if isempty(row)
                cmd = [];
                return
            end
            cmd = g1motor.MotorCmd('id', row(1), 'mode', row(2), ...
                'tau', row(3), 'vel', row(4), 'pos', row(5), 'kp', row(6), ...
                'kd', row(7), 'flag', row(8) ~= 0, 'byte3', row(9));
        end
    end
end

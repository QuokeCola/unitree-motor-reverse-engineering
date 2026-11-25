%> @file MotorState.m
%> @brief Reply frame.

%> @brief One reply frame (motor to controller).
classdef MotorState
    properties
        %> Motor id.
        id = 0
        %> Mode the motor is actually in.
        mode = 0
        %> Net applied torque, int16.
        tau = 0
        %> Velocity; 1 count = 128 position counts per second.
        vel = 0
        %> Position; 32768 counts per rotor turn.
        pos = 0
        %> Winding temperature, deg C.
        temp_coil = 0
        %> Shell temperature, deg C.
        temp_shell = 0
        %> Bus voltage, 0.5 V per count.
        voltage_raw = 0
        %> Status word; 0 in normal operation.
        error = 0
        %> Unknown.
        word18 = 0
        %> Unknown.
        byte20 = 0
        %> Unknown.
        byte21 = 0
    end

    methods
        %> @brief Bus voltage.
        %> @return Volts.
        function volts = voltage(state)
            volts = state.voltage_raw * 0.5;
        end

        %> @brief Whether the motor reports a fault.
        %> @return True if the status word is non-zero.
        function is_fault = fault(state)
            is_fault = state.error ~= 0;
        end
    end

    methods (Static)
        %> @brief Parses a reply frame.
        %> @param frame Exactly 26 bytes (uint8).
        %> @return The state, or [] if the header or checksum is wrong.
        function state = decode(frame)
            state = g1motor.MotorState.from_rows( ...
                g1motor_mex('decode', uint8(frame)));
        end

        %> @brief Builds states from the rows returned by the MEX gateway.
        %> @param rows N-by-12 matrix.
        %> @return 1-by-N MotorState array, or [] for no rows.
        function states = from_rows(rows)
            states = [];
            names = {'id', 'mode', 'tau', 'vel', 'pos', 'temp_coil', ...
                     'temp_shell', 'voltage_raw', 'error', 'word18', ...
                     'byte20', 'byte21'};
            for k = size(rows, 1):-1:1
                state = g1motor.MotorState;
                for n = 1:numel(names)
                    state.(names{n}) = rows(k, n);
                end
                if isempty(states), states = state; end
                states(k) = state;
            end
        end

        %> @brief Builds a map from motor id to state.
        %> @param rows N-by-12 matrix from the MEX gateway.
        %> @return containers.Map from id to MotorState.
        function map = to_map(rows)
            map = containers.Map('KeyType', 'double', 'ValueType', 'any');
            states = g1motor.MotorState.from_rows(rows);
            for k = 1:numel(states)
                map(states(k).id) = states(k);
            end
        end
    end
end

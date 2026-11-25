%> @file Joint.m
%> @brief Joint-level view of a motor: gear ratio, end stops, software zero.

%> @brief Gear ratio, end stops and zero of one joint, in raw counts.
%>
%> After a power cycle or restart a motor's position reading can come back
%> shifted by a whole number of output-shaft turns. sync() moves the stored
%> zero and end stops by the same amount.
classdef Joint < handle
    properties
        %> Rotor turns per output turn.
        gear_ratio = 1
        %> Lower end stop, or [].
        low = []
        %> Upper end stop, or [].
        high = []
        %> Count that is angle 0; defaults to the middle of the travel.
        zero = 0
        %> Counts kept clear of each end stop by clamp().
        margin = 0
        %> Slack allowed by sync().
        tolerance = 3000
    end

    methods
        %> @brief Creates a joint, with or without end stops.
        %> @param gear_ratio Rotor turns per output turn.
        %> @param low Lower end stop in counts (optional).
        %> @param high Upper end stop in counts (optional).
        function joint = Joint(gear_ratio, low, high)
            joint.gear_ratio = gear_ratio;
            if nargin == 2
                error('g1motor:ValueError', 'give both end stops or neither');
            end
            if nargin == 3
                if low >= high
                    error('g1motor:ValueError', 'low must be below high');
                end
                joint.low = low;
                joint.high = high;
                joint.zero = round((low + high) / 2);
            end
        end

        %> @brief Size of one output turn.
        %> @return Position counts in one turn of the output shaft.
        function counts = turn_counts(joint)
            counts = round(joint.gear_ratio * g1motor.COUNTS_PER_ROTOR_TURN);
        end

        %> @brief Angle between the end stops.
        %> @return Travel in rad, or [] without end stops.
        function rad = travel(joint)
            rad = [];
            if ~isempty(joint.low)
                rad = (joint.high - joint.low) / joint.turn_counts() * 2 * pi;
            end
        end

        %> @brief Shifts zero and end stops by whole output turns to match a
        %>        position reading.
        %>
        %> Call after every power-on or restart.
        %> @param pos Current position reading.
        %> @return Number of output turns shifted.
        function turns = sync(joint, pos)
            if isempty(joint.low)
                reference = joint.zero;
            else
                reference = (joint.low + joint.high) / 2;
            end
            turns = round((pos - reference) / joint.turn_counts());
            shift = turns * joint.turn_counts();
            joint.zero = joint.zero + shift;
            if ~isempty(joint.low)
                joint.low = joint.low + shift;
                joint.high = joint.high + shift;
                if pos < joint.low - joint.tolerance || ...
                        pos > joint.high + joint.tolerance
                    error('g1motor:ValueError', ...
                          'position %d does not fit the end stops', pos);
                end
            end
        end

        %> @brief Converts a position to an output-shaft angle.
        %> @param pos Position in counts.
        %> @return Angle in rad from the zero.
        function rad = angle(joint, pos)
            rad = (pos - joint.zero) / joint.turn_counts() * 2 * pi;
        end

        %> @brief Converts an output-shaft angle to a position.
        %> @param angle Angle in rad from the zero.
        %> @return Position in counts.
        function pos = counts(joint, angle)
            pos = round(joint.zero + angle / (2 * pi) * joint.turn_counts());
        end

        %> @brief Converts a velocity reading to output-shaft speed.
        %> @param vel Velocity in counts.
        %> @return Speed in rad/s.
        function rads = speed(joint, vel)
            rads = vel * 128 / joint.turn_counts() * 2 * pi;
        end

        %> @brief Limits a position to the end stops less the margin.
        %> @param pos Position in counts.
        %> @return The limited position.
        function pos = clamp(joint, pos)
            pos = round(pos);
            if ~isempty(joint.low)
                pos = round(min(max(pos, joint.low + joint.margin), ...
                                joint.high - joint.margin));
            end
        end
    end
end

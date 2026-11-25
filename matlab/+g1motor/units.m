%> @file units.m
%> @brief Conversion between raw counts and physical units, on the rotor side.

%> @brief Scales and conversions for position, velocity and torque.
%>
%> Position (32768 counts per rotor turn) is supported by data: after a
%> restart the reading jumps by exactly 16 or 25 rotor turns. Velocity follows
%> from it and the measured ratio of 128. The torque scale is assumed from the
%> public GO-M8010-6 protocol and not checked. Use Joint for output angles.
classdef units
    properties (Constant)
        %> Radians of rotor angle per position count.
        POS_RAD_PER_COUNT = 2 * pi / 32768
        %> Radians per second of rotor speed per velocity count.
        VEL_RADS_PER_COUNT = 2 * pi / 256
        %> Newton metres of rotor torque per torque count (assumed).
        TAU_NM_PER_COUNT = 1 / 256
        %> Position counts per second represented by one velocity count.
        POS_COUNTS_PER_VEL_COUNT = 128
    end

    methods (Static)
        %> @brief Converts a position to rotor angle.
        %> @param counts Position in counts.
        %> @return Angle in rad.
        function rad = pos_to_rad(counts)
            rad = counts * g1motor.units.POS_RAD_PER_COUNT;
        end

        %> @brief Converts a rotor angle to a position.
        %> @param rad Angle in rad.
        %> @return Position in counts.
        function counts = rad_to_pos(rad)
            counts = round(rad / g1motor.units.POS_RAD_PER_COUNT);
        end

        %> @brief Converts a velocity reading to rotor speed.
        %> @param counts Velocity in counts.
        %> @return Speed in rad/s.
        function rads = vel_to_rads(counts)
            rads = counts * g1motor.units.VEL_RADS_PER_COUNT;
        end

        %> @brief Converts a rotor speed to a velocity command.
        %> @param rads Speed in rad/s.
        %> @return Velocity in counts.
        function counts = rads_to_vel(rads)
            counts = round(rads / g1motor.units.VEL_RADS_PER_COUNT);
        end

        %> @brief Velocity command for a position target that moves at a
        %>        given rate.
        %> @param counts_per_second Rate of the position target.
        %> @return Velocity in counts.
        function counts = pos_rate_to_vel(counts_per_second)
            counts = round(counts_per_second / ...
                           g1motor.units.POS_COUNTS_PER_VEL_COUNT);
        end

        %> @brief Converts a torque reading to rotor torque (assumed scale).
        %> @param counts Torque in counts.
        %> @return Torque in N*m.
        function nm = tau_to_nm(counts)
            nm = counts * g1motor.units.TAU_NM_PER_COUNT;
        end

        %> @brief Converts a rotor torque to a torque command (assumed scale).
        %> @param nm Torque in N*m.
        %> @return Torque in counts.
        function counts = nm_to_tau(nm)
            counts = round(nm / g1motor.units.TAU_NM_PER_COUNT);
        end
    end
end

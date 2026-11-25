%> @file find_reply.m
%> @brief Search of a receive buffer for a reply frame.

%> @brief Finds the first valid reply in a receive buffer.
%>
%> Skips echoed command bytes and noise.
%> @param buffer uint8 bytes received so far.
%> @return The reply as a MotorState, or [] if none is complete and valid.
function state = find_reply(buffer)
state = g1motor.MotorState.from_rows(g1motor_mex('find_reply', uint8(buffer)));
end

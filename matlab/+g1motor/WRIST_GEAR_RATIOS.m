%> @file WRIST_GEAR_RATIOS.m
%> @brief Constant WRIST_GEAR_RATIOS.

%> @brief Gear ratios seen on one G1 wrist (ids 4, 5, 6); id 5 is assumed.
%> @return containers.Map from motor id to gear ratio.
function value = WRIST_GEAR_RATIOS()
value = containers.Map({4, 5, 6}, {16, 25, 25});
end

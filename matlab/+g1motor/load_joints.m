%> @file load_joints.m
%> @brief Reader of end-stop files.

%> @brief Reads joints from an end-stop file written by `g1motor limits`.
%> @param path JSON file of the form {"4": [low, high], ...}.
%> @param gear_ratios containers.Map of gear ratio per id; defaults to
%>        WRIST_GEAR_RATIOS.
%> @param margin Margin given to every joint; default 0.
%> @return containers.Map from motor id to Joint.
function joints = load_joints(path, gear_ratios, margin)
if nargin < 2, gear_ratios = g1motor.WRIST_GEAR_RATIOS; end
if nargin < 3, margin = 0; end
text = fileread(path);
tokens = regexp(text, '"(\d+)"\s*:\s*\[\s*(-?\d+)\s*,\s*(-?\d+)\s*\]', ...
                'tokens');
joints = containers.Map('KeyType', 'double', 'ValueType', 'any');
for k = 1:numel(tokens)
    id = str2double(tokens{k}{1});
    joint = g1motor.Joint(gear_ratios(id), str2double(tokens{k}{2}), ...
                          str2double(tokens{k}{3}));
    joint.margin = margin;
    joints(id) = joint;
end
end

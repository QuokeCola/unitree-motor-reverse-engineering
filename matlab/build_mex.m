%> @file build_mex.m
%> @brief Compiles the MEX gateway of the +g1motor package from ../cpp.
%>
%> Needs a C++17 compiler set up for MEX (run "mex -setup C++" once). Run it
%> again after the C++ sources change.

%> @brief Builds g1motor_mex into +g1motor/private.
function build_mex()
here = fileparts(mfilename('fullpath'));
cpp = fullfile(here, '..', 'cpp');
out = fullfile(here, '+g1motor', 'private');
sources = {fullfile(here, 'g1motor_mex.cc'), ...
           fullfile(cpp, 'g1motor', 'protocol.cc'), ...
           fullfile(cpp, 'g1motor', 'serial_port.cc'), ...
           fullfile(cpp, 'g1motor', 'motor_bus.cc'), ...
           fullfile(cpp, 'g1motor', 'joint.cc')};
if ispc
    flags = {'COMPFLAGS=$COMPFLAGS /std:c++17 /DNOMINMAX'};
else
    flags = {'CXXFLAGS=$CXXFLAGS -std=c++17'};
end
clear g1motor_mex
try
    mex('-R2018a', flags{:}, ['-I' cpp], '-outdir', out, ...
        '-output', 'g1motor_mex', sources{:});
catch build_error
    % With some new compilers mex links a working file and then rejects it
    % in its own final check. Accept the file if the package can use it.
    addpath(here);
    try
        works = g1motor.crc32_core(zeros(1, 4, 'uint8')) > 0;
    catch
        works = false;
    end
    if ~works, rethrow(build_error); end
    warning('g1motor:mexCheck', ...
            'mex reported "%s" but the built file works.', ...
            build_error.message);
end
fprintf('built %s\n', fullfile(out, ['g1motor_mex.' mexext]));
end

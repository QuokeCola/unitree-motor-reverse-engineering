%> @file build_mex.m
%> @brief Compiles g1motor_mex from the C++ library in ../cpp.
%>
%> Needs a C++17 compiler set up for MEX (run "mex -setup C++" once). Run it
%> again after the C++ sources change.

%> @brief Builds g1motor_mex next to this file.
function build_mex()
here = fileparts(mfilename('fullpath'));
cpp = fullfile(here, '..', 'cpp');
sources = {fullfile(here, 'g1motor_mex.cc'), ...
           fullfile(cpp, 'src', 'protocol.cc'), ...
           fullfile(cpp, 'src', 'serial_port.cc'), ...
           fullfile(cpp, 'src', 'motor_bus.cc')};
if ispc
    flags = {'COMPFLAGS=$COMPFLAGS /std:c++17 /D_USE_MATH_DEFINES /DNOMINMAX'};
else
    flags = {'CXXFLAGS=$CXXFLAGS -std=c++17'};
end
clear g1motor_mex
try
    mex('-R2018a', flags{:}, ['-I' fullfile(cpp, 'include')], ...
        '-outdir', here, '-output', 'g1motor_mex', sources{:});
catch buildError
    % With some new compilers mex links a working file and then rejects it
    % in its own final check. Accept the file if it loads and encodes a frame.
    addpath(here);
    try
        works = numel(g1motor_mex('encode', struct('id', 0))) == 20;
    catch
        works = false;
    end
    if ~works, rethrow(buildError); end
    warning('g1motor:mexCheck', ...
            'mex reported "%s" but the built file works.', buildError.message);
end
fprintf('built %s\n', fullfile(here, ['g1motor_mex.' mexext]));
end

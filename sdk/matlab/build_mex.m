function build_mex()
%BUILD_MEX Compile g1motor_mex from the C++ library sources next to this folder.
%   Needs a C++17 compiler set up for MEX (run "mex -setup C++" once). Run it again after
%   updating the C++ sources.
here = fileparts(mfilename('fullpath'));
cpp = fullfile(here, '..', 'cpp');
src = {fullfile(here, 'g1motor_mex.cpp'), fullfile(cpp, 'src', 'protocol.cpp'), ...
       fullfile(cpp, 'src', 'serial_port.cpp'), fullfile(cpp, 'src', 'bus.cpp')};
if ispc
    flags = {'COMPFLAGS=$COMPFLAGS /std:c++17 /D_USE_MATH_DEFINES /DNOMINMAX'};
else
    flags = {'CXXFLAGS=$CXXFLAGS -std=c++17'};
end
clear g1motor_mex
mex('-R2018a', flags{:}, ['-I' fullfile(cpp, 'include')], '-outdir', here, '-output', 'g1motor_mex', src{:});
fprintf('built %s\n', fullfile(here, ['g1motor_mex.' mexext]));
end

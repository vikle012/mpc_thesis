function M_lut = WHTC_M_table(time, torque_denormalized)
% FUNCTION: (( NOT USED ))
%           Look-up table for torque profile for WHTC.
%
% SYNTAX: [output1] = WHTC_M_table(input1, input2);
%
% INPUTS:
%   input1 - Time vector
%   input2 - Torque vector with denormalized values
%
% OUTPUTS:
%   output1 - Lookup table for denormalized torque profile
%
% EXAMPLE:
%   [param.M_lut] = WHTC_M_table(time, torque_denormalized);
%
% OTHER FILES REQUIRED:
%   .m files:
%       WHTC_N_table
%       WHTC_dN_table
%   	WHTC_M_table
%
%   .mat files:
%       WHTC_data
%
% SUBFUNCTIONS:
%   none
%
% Author: Hampus Andersson and Fredrik Andersson
% Mars 2018; Last revision: 23-May-2018

%------------- BEGIN CODE --------------

tic
disp('Creating look-up-table for WHTC torque profile...');

K = 8;
time2 = linspace(0, time(end), length(time) * K + 1);
torque2 = interp1([0; time], [0; torque_denormalized], time2, 'linear');

M_lut = casadi.interpolant('LUT', 'bspline', {time2}, torque2);
fprintf('Finished in %.0f seconds \n \n', toc);

%------------- END OF CODE --------------
end


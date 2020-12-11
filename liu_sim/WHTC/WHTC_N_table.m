function lut = WHTC_N_table(time, speed)
% FUNCTION: Look-up table for enigne speed profile for WHTC
%
% SYNTAX: output1 = WHTC_N_table(input1, input2);
%
% INPUTS:
%   input1 - time vector for WHTC
%   input2 - denormalized speed vector for WHTC
%
% OUTPUTS:
%   output1 - look-up table for the normalized speed profile for WHTC
%
% EXAMPLE:
%   [param.lut] = WHTC_N_table(time, speed_scaled);
%
% OTHER FILES REQUIRED:
%   .m files:
%       none
%
%   .mat files:
%       none
%
% SUBFUNCTIONS:
%   none
%
% Author: Hampus Andersson and Fredrik Andersson
% Mars 2018; Last revision: 23-May-2018

%------------- BEGIN CODE --------------

tic
disp('Creating look-up-table for WHTC speed profile...');

% Increase data density for precision of lookup table
K = 4;
time2 = linspace(0, time(end), length(time) * K + 1);
speed2 = interp1([0; time], [speed(1); speed], time2, 'linear');

% Create lookup table using CasADi
lut = casadi.interpolant('LUT', 'bspline', {time2}, speed2);
fprintf('Finished in %.0f seconds \n \n', toc);

%------------- END OF CODE --------------
end


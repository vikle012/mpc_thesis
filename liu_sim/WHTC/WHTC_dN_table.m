function lut = WHTC_dN_table(time, speed)
% FUNCTION: Creates a lookup table of speed derivatives (acceleration) 
%           for use as dynamics in speed state.
%
% SYNTAX: output1 = WHTC_dN_tabel(input1, input2);
%
% INPUTS:
%   input1 - time vector for WHTC
%   input2 - denormalized speed vector for WHTC
%
% OUTPUTS:
%   output1 - look-up table for acceleration profile for WHTC
%
% EXAMPLE:
%   [param.lut] = WHTC_dN_table(time, speed_scaled);
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
% Mars 2018; Last revision: 28-Mars-2018

%------------- BEGIN CODE --------------

tic
disp('Creating look-up-table for WHTC acceleration profile...');

acceleration = zeros(length(time),1);   % preallocate vector (zero end)

% Estimate speed derivative for each interval
for i = 1:length(time)-1
    acceleration(i) = (speed(i+1)-speed(i))./(time(i+1)-time(i));
end

K = 4;
time2 = linspace(0, time(end), length(time) * K + 1);
acceleration2 = interp1([0; time], [0; acceleration], time2, 'linear');

lut = casadi.interpolant('LUT', 'bspline', {time2}, acceleration2);
fprintf('Finished in %.0f seconds \n \n', toc);

%------------- END OF CODE --------------
end
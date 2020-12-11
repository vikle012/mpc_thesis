function lut = WHTC_M_sparse_table(time, torque_denormalized)
% FUNCTION: Construct a lookup table for the denormalized torque profile
%           for the WHTC using a custom technique to increase resolution 
%           while minimizing complexity of function. 4 data points are 
%           spread linearly from an initial data points for interpolation.
%
% SYNTAX: [output1] = WHTC_M_sparse_table(input1, input2);
%
% INPUTS:
%   input1 - Time vector for WHTC
%   input2 - Denormalized torque vector, same length as input1.
%
% OUTPUTS:
%   output1 - Lookup table for WHTC denormalized torque profile
%
% EXAMPLE:
%   M_lut = WHTC_M_sparse_table(time, torque_denorm);
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
disp('Creating look-up-table for WHTC torque profile...');

% Time-step for discretization and data points in each direction from
% original data point
dt = 0.002;
steps = 4;

% Finer discretization with dt time-step and interpolating the torque
time2 = 0:dt:1801;
torque_scaled2 = interp1([0; time; 1801], [0; torque_denormalized; 0], time2, 'linear');

% Looping through original data points to add 'steps' many data points in
% each direction from the original point
vector = 0;
time_v = 0;
for i = 0:1800
    [~, pos] = min(abs(time2 - i));
    for j = -steps:1:steps
        if pos+j <= 1
        else
            vector = [vector torque_scaled2(min(max(1,pos+j), length(time2)))];
            time_v = [time_v time2(min(max(1,pos+j), length(time2)))];
        end
    end
end

% Creating look-up table using CasADi.
lut = casadi.interpolant('LUT', 'bspline', {time_v}, vector);
fprintf('Finished in %.0f seconds \n \n', toc);

%------------- END OF CODE --------------
end


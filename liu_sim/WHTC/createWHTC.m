function WHTC = createWHTC(engine_map, regen_brake)
% FUNCTION: Creates all look-up tables and parameters for WHTC cycle
%
% SYNTAX: [output1] = create_WHTC(input1);
%
% INPUTS:
%   input1 - engine_map struct for liu_diesel_2
%
% OUTPUTS:
%   output1 - Struct with alla data and look-up tables for WHTC
%
% EXAMPLE:
%   param.WHTC = create_WHTC(engine_map);
%
% OTHER FILES REQUIRED:
%   .m files:
%       WHTC_engine_speed_values
%       WHTC_N_table
%       WHTC_dN_table
%   	WHTC_M_sparse_table
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
% If called from ld2_NOx, no regenerative braking is available
switch nargin
    case 1
        regen_brake = 0;
end

% Loading data for WHTC cycle
load('WHTC_data.mat', 'time', 'speed', 'torque');

% Denormalizing engine speed for the specific engine.
%speed_scaled = speed .* 18.46 + 500;
[n_lo, n_pref, n_hi] = WHTC_engine_speed_values(engine_map);
speed_scaled = speed.*(0.45.*n_lo + 0.45.*n_pref + 0.1.*n_hi - 500) ...
    .*2.0327./100 + 500;

% Creating look-up tables for engine speed, derivative of
% engine speed and for torque.
WHTC.lut_N      = WHTC_N_table(time, speed_scaled);
WHTC.lut_dN     = WHTC_dN_table(time, speed_scaled);

% Maximum and minimum torque dependent of engine speed. M_ice_min
% is a fitted polynomial for pumping and friction torque in engine.
M_ice_max = interp1(engine_map.rpm_range, engine_map.torque_grid(end,:), speed_scaled)';
M_ice_min = (-3.356e-5.*speed_scaled.^2 + 0.002508.*speed_scaled - 76.06 + 1)';

% Denormalizing torque for the specific engine
torque_scaled = torque; % For preallocating memory
for i = 1:length(time)
    % Producing torque
    torque_scaled(i) = torque(i) .* M_ice_max(i) ./ 100;
    
    % Engine brake and regenerative braking
    if torque_scaled(i) < 0
        M_em_min = max(M_em_min_fn(0, speed_scaled(i)));
        
        % Saturate regenerative braking
        if M_em_min < -regen_brake
            M_em_min = -regen_brake;
        end
        torque_scaled(i) = M_ice_min(i) + M_em_min;
    end
end

WHTC.lut_M = WHTC_M_sparse_table(time, torque_scaled);

%Adding data to struct
WHTC.time       = time;
WHTC.speed      = speed;
WHTC.torque     = torque;
WHTC.speed_scaled   = speed_scaled;
WHTC.torque_scaled  = torque_scaled;

%------------- END OF CODE --------------
end
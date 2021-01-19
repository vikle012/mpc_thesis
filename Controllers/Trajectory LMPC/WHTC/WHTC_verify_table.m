% NAME: WHTC_verify_table
%
% PURPOSE:  Plotting denormlized torque profile and verify lookup table
%           trough plots.
%
% OTHER FILES REQUIRED:
%   .m files:
%       WHTC_engine_speed_values
%       WHTC_M_sparse_table
%
%   .mat files:
%       WHTC_data
%       ld2_eng_map
%
% SUBFUNCTIONS:
%   none
%
% Author: Hampus Andersson and Fredrik Andersson
% May 2018; Last revision: 23-May-2018

%------------- BEGIN CODE --------------

%% Load data

load('WHTC_data.mat', 'time', 'speed', 'torque');
load('ld2_eng_map.mat', 'engine_map')

%% Denormalizing engine speed for the specific engine.

% From [0, 100] to [500, 2400] rpm
[n_lo, n_pref, n_hi] = WHTC_engine_speed_values(engine_map);
speed_scaled = speed.*(0.45.*n_lo + 0.45.*n_pref + 0.1.*n_hi - 500) ...
    .*2.0327./100 + 500;

% Maximum and minimum torque dependent of engine speed. M_ice_min
% is a fitted polynomial for pumping and friction torque in engine.
M_ice_max = interp1(engine_map.rpm_range, engine_map.torque_grid(end,:), speed_scaled)';
M_ice_min = (-3.356e-5.*speed_scaled.^2 + 0.002508.*speed_scaled - 76.06 + 1)';


% Denormalizing torque for the specific engine
torque_scaled = torque; % For preallocating memory
for i = 1:length(time)
    torque_scaled(i) = torque(i) .* M_ice_max(i) ./ 100;
    
    if torque_scaled(i) < 0
        M_em_min = max(M_em_min_fn(0, speed_scaled(i)));
        if M_em_min < -1000
            M_em_min = -1000;
        end
        torque_scaled(i) = M_ice_min(i) + M_em_min;
    end
end

%% Plotting

M_lut = WHTC_M_sparse_table(time, torque_scaled);

% Plot with better precision
time_range = linspace(0, time(end), time(end) * 20);
%time_range = linspace(440, 500, time(end) * 20);

% Preallocating for faster loop, takes time anyhow
M = time_range;
for t = 1:length(time_range)
    M(t) = full(M_lut(time_range(t)));
end

figure; hold on;
plot(time, torque_scaled, 'k', 'LineWidth', 2)
plot(time_range, M, 'r')
legend('WHTC','look-up-table');
xlabel('Time (s)');
ylabel('Torque (Nm)');
title('WHTC Denormalized Torque Profile');

%% Plotting fitting error

figure; hold on;
plot(time(440:500), torque_scaled(440:500), 'k', 'LineWidth', 2)
plot(time_range, M, 'r')
plot(time_v, vector, 'bo')
title('WHTC Torque profile (peak at t = 486s)');
legend('WHTC','Look-up table','Data points for interpolant','Location','NW');
ylabel('Torque (Nm)');
xlabel('Time (s)')
ylim([2290 2350]);
xlim([485.8 486.1]);


%------------- END OF CODE --------------





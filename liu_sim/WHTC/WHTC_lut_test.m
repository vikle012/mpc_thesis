% NAME: WHTC_lut_test
%
% PURPOSE: Testing the precision of the lookup tables produced with:
%               -> WHTC_M_table (not preferred)
%               -> WHTC_N_table
%               -> WHTC_M_sparse_table (preferred)
%
% OTHER FILES REQUIRED:
%   .m files:
%       WHTC_engine_speed_values
%       WHTC_N_table
%   	WHTC_M_table
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

load('WHTC_data.mat');
load('ld2_eng_map.mat');

% Denormalize speed and torque profile
[n_lo, n_pref, n_hi] = WHTC_engine_speed_values(engine_map);
speed_denormalized = speed.*(0.45.*n_lo + 0.45.*n_pref + 0.1.*n_hi - 500) ...
    .*2.0327./100 + 500;

% Polynomial for maximum engine breaking
min_Nm_per_speed = -3.356e-5.*speed_denormalized.^2 + 0.002508.*speed_denormalized - 76.06 + 1;

% Maximum and minimum torque dependent of engine speed. M_ice_min
% is a fitted polynomial for pumping and friction torque in engine.
M_ice_max = interp1(engine_map.rpm_range, engine_map.torque_grid(end,:), speed_denormalized)';
M_ice_min = (-3.356e-5.*speed_denormalized.^2 + 0.002508.*speed_denormalized - 76.06 + 1)';

% Denormalizing torque for the specific engine
torque_denormalized = torque; % For preallocating memory
for i = 1:length(time)
    torque_denormalized(i) = torque(i) .* M_ice_max(i) ./ 100;
    
    if torque_denormalized(i) < 0
        M_em_min = max(M_em_min_fn(0, speed_denormalized(i)));
        if M_em_min < -1000
            M_em_min = -1000;
        end
        torque_denormalized(i) = M_ice_min(i) + M_em_min;
    end
end

%% Plot distrubition of operating points in engine map

figure; hold on;
plot(speed_denormalized, torque .*23.9, 'r*')
plot(speed_denormalized, M_ice_min, 'r-', 'LineWidth', 2)
plot(speed_denormalized, torque_denormalized, 'k*')
plot(speed_denormalized, M_ice_max, 'r.', 'LineWidth', 2)

xlim([500 2400]);

%% Plot lookup tables for WHTC speed and torque profile.

M_lut = WHTC_M_sparse_table(time, torque_denormalized); % GOOD PRECISION
%M_lut = WHTC_M_table(time, torque_scaled); % BAD PRECISION
N_lut = WHTC_N_table(time, speed_denormalized);

% Plot with better precision
time_vector = linspace(0, time(end), time(end) * 30);

tic
% Preallocating for faster loop
M = time_vector;
N = time_vector;
for t = 1:length(time_vector)
    M(t) = full(M_lut(time_vector(t)));
    N(t) = full(N_lut(time_vector(t)));
end
toc

figure; hold on;
plot(time, torque_denormalized, 'k', 'LineWidth', 2)
plot(time_vector, M, 'r')
legend('WHTC','look-up-table');

figure; hold on;
plot(time, speed_denormalized, 'k', 'LineWidth', 2)
plot(time_vector, N, 'r')

%------------- END OF CODE --------------
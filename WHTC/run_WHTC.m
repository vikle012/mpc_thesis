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
engine_torque_map

%% Denormalizing engine speed for the specific engine.

% From [0, 100] to [500, 2000] rpm
n_idle = 500;
n_lo = 815;
n_pref = 1418;
n_hi = 2301;
speed_scaled = speed.*(0.45.*n_lo + 0.45.*n_pref + 0.1.*n_hi - 500) ...
    .*2.0327./100 + 500;

% Maximum and minimum torque dependent of engine speed. M_ice_min
% is a fitted polynomial for pumping and friction torque in engine.
M_e_max = interp1(N_e, M_e, speed_scaled)';

% Denormalizing torque for the specific engine
torque_scaled = torque; % For preallocating memory
for i = 1:length(time)
    torque_scaled(i) = torque(i) .* M_e_max(i) ./ 100;
end

%% Plotting

h = figure;
subplot(2,1,1)
plot(time, torque_scaled)
ylabel('Engine torque [Nm]')
xlabel('Time [s]')

subplot(2,1,2)
hold on
plot(time, speed_scaled)
ylabel('Engine speed [rpm]')
xlabel('Time [s]')

%% Export as PDF
set(h,'Units','Inches');
pos = get(h,'Position');
set(h,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
print(h,'Figures/WHTC_denorm_values','-dpdf','-r0')

%------------- END OF CODE --------------





%% Load data

load('WHTC_data.mat')
max_torque_curve % Runs the file

%% Denormalizing engine speed for the specific engine

% Values from Hampus Andersson and Fredrik Andersson
% Engine where n_e ranges from 500 to 2400 rpm
n_idle = 500;
n_lo = 815;
n_pref = 1418;
n_hi = 2301;

% Convert to examined engine where n_e ranges from 500 to 2000 rpm
% Conversion done by translation + scaling + re-translation
n_idle = (n_idle - 500)*1500/1900 + 500;
n_lo = (n_lo - 500)*1500/1900 + 500;
n_pref = (n_pref - 500)*1500/1900 + 500;
n_hi = (n_hi - 500)*1500/1900 + 500;

% De-normalization
actual_speed = speed.*(0.45.*n_lo + 0.45.*n_pref + 0.1.*n_hi - n_idle) ...
    .*2.0327./100 + n_idle;

% Maximum torque dependent of engine speed from max_torque_curve.m
M_e_max = interp1(N_e, M_e, actual_speed)';

% Denormalizing torque for the specific engine
actual_torque = torque; % For preallocating memory
for i = 1:length(time)
    actual_torque(i) = torque(i) .* M_e_max(i) ./ 100;
end 

%% Plot

h = figure;
subplot(2,1,1)
plot(time, actual_torque)
ylabel('Engine torque [Nm]')
xlabel('Time [s]')

subplot(2,1,2)
hold on
plot(time, actual_speed)
ylabel('Engine speed [rpm]')
xlabel('Time [s]')

%% Export as PDF

% set(h,'Units','Inches');
% pos = get(h,'Position');
% set(h,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
% print(h,'Figures/WHTC_denorm_values','-dpdf','-r0')
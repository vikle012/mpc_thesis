% NAME: compile_high_operating_point
%
% PURPOSE:  Load and combine runs made with fuel and light-off time as
%           objective. Compare them with runs made with NOx Cu-Zeolite and 
%           Fe-Zeolite optimal. Plot interesting things. Steady state at
%           1100 rpm and 800 Nm
%
% OTHER FILES REQUIRED:
%   .m files:
%       createParam__hev_eats_ss
%
%   .mat files:
%       ld2_eng_map
%       kallstart_EATS_HEV_0_180_K1d4
%       kallstart_EATS_HEV_180_1200_K1d4_fuel_koppla_efter
%       light_off_kallstart_EATS_ld2_NOx_0_248_K1d4
%       light_off_kallstart_EATS_ld2_NOx_248_1200_K1d4_koppla_efter
%       kallstart_EATS_HEV_0_1200_K600d4_TP_Cu
%       high_kallstart_EATS_HEV_0_1200_K300d4_TP_Fe
%
% SUBFUNCTIONS:
%   none
%
% Author: Hampus Andersson and Fredrik Andersson
% May 2018; Last revision: 05-June-2018

%------------- BEGIN CODE --------------

%% LOAD PARAM

load('ld2_eng_map.mat', 'engine_map')

% Only create 'param' if it does not already exist to save time
% Param is neede for some plots
if ~exist('param', 'var')
    regen_brake = 1000; % Nm
    param = createParam__hev_eats_ss(engine_map, regen_brake);
end

%% LOAD RUNS

% HYBRID STANDARD
light_off_hybrid = load_run('kallstart_EATS_HEV_0_180_K1d4',0);
fuel_hybrid = load_run('kallstart_EATS_HEV_180_1200_K1d4_fuel_koppla_efter',0);

% CONVENTIONAL
light_off_conventional = load_run('light_off_kallstart_EATS_ld2_NOx_0_248_K1d4',0);
fuel_conventional = load_run('light_off_kallstart_EATS_ld2_NOx_248_1200_K1d4_koppla_efter',0);

% Cu-Zeolite OPTIMAL
NOx_Cu_optimal = load_run('kallstart_EATS_HEV_0_1200_K600d4_TP_Cu',0);

% Fe-Zeolite OPTIMAL
NOx_Fe_optimal = load_run('high_kallstart_EATS_HEV_0_1200_K300d4_TP_Fe',0);

%% COMBINE DATA FROM RUNS

% Time
time_hybr = [light_off_hybrid.t_vec.val(1:end-1), ...
    fuel_hybrid.t_vec.val + light_off_hybrid.t_vec.val(end)];
time_conv = [light_off_conventional.t_vec.val(1:end-1), ...
    fuel_conventional.t_vec.val + light_off_conventional.t_vec.val(end)];
time_NOx_Cu_optimal = NOx_Cu_optimal.t_vec.val;
time_NOx_Fe_optimal = NOx_Fe_optimal.t_vec.val;

% x-vec
xvec_hybr = [light_off_hybrid.x_vec.val(1:end-1,:); ...
    fuel_hybrid.x_vec.val];
xvec_conv = [light_off_conventional.x_vec.val(1:end-1,:); ...
    fuel_conventional.x_vec.val];
xvec_NOx_Cu_optimal = NOx_Cu_optimal.x_vec.val;
xvec_NOx_Fe_optimal = NOx_Fe_optimal.x_vec.val;

% NOx Cu and Fe
hybr_NOx_Cu_instant = [light_off_hybrid.NOx_TP_Cu.val(1:end-1), ...
    fuel_hybrid.NOx_TP_Cu.val];
conv_NOx_Fe_instant = [light_off_hybrid.NOx_TP_Fe.val(1:end-1), ...
    fuel_hybrid.NOx_TP_Fe.val];
Cu_opt_NOx_Cu_instant = [light_off_conventional.NOx_TP_Cu.val(1:end-1), ...
    fuel_conventional.NOx_TP_Cu.val];
NOx_Fe_instant2 = [light_off_conventional.NOx_TP_Fe.val(1:end-1), ...
    fuel_conventional.NOx_TP_Fe.val];
NOx_Cu_instant3 = NOx_Cu_optimal.NOx_TP_Cu.val;
NOx_Fe_instant3 = NOx_Cu_optimal.NOx_TP_Fe.val;
NOx_Cu_instant4 = NOx_Fe_optimal.NOx_TP_Cu.val;
NOx_Fe_instant4 = NOx_Fe_optimal.NOx_TP_Fe.val;

% Integrate NOx Cu and Fe
NOx_Cu1 = zeros(length(time_hybr),1);
NOx_Fe1 = zeros(length(time_hybr),1);
NOx_Cu2 = zeros(length(time_conv),1);
NOx_Fe2 = zeros(length(time_conv),1);
NOx_Cu3 = zeros(length(time_NOx_Cu_optimal),1);
NOx_Fe3 = zeros(length(time_NOx_Cu_optimal),1);
NOx_Cu4 = zeros(length(time_NOx_Fe_optimal),1);
NOx_Fe4 = zeros(length(time_NOx_Fe_optimal),1);
for i = 2:length(time_hybr)
    NOx_Cu1(i) = trapz(time_hybr(1:i),hybr_NOx_Cu_instant(1:i));
    NOx_Fe1(i) = trapz(time_hybr(1:i),conv_NOx_Fe_instant(1:i));
end
for i = 2:length(time_conv)
    NOx_Cu2(i) = trapz(time_conv(1:i),Cu_opt_NOx_Cu_instant(1:i));
    NOx_Fe2(i) = trapz(time_conv(1:i),NOx_Fe_instant2(1:i));
end
for i = 2:length(time_NOx_Cu_optimal)
    NOx_Cu3(i) = trapz(time_NOx_Cu_optimal(1:i), NOx_Cu_instant3(1:i)); 
    NOx_Fe3(i) = trapz(time_NOx_Cu_optimal(1:i), NOx_Fe_instant3(1:i));
end
for i = 2:length(time_NOx_Fe_optimal)
    NOx_Cu4(i) = trapz(time_NOx_Fe_optimal(1:i), NOx_Cu_instant4(1:i)); 
    NOx_Fe4(i) = trapz(time_NOx_Fe_optimal(1:i), NOx_Fe_instant4(1:i));
end

%% PRINT DATA FOR COMPARISON

% Combine data from LO optimal and fuel optimal, hybrid
hybr_NOx_Cu = [light_off_hybrid.NOx_TP_Cu.val(1:end-1), ...
    fuel_hybrid.NOx_TP_Cu.val];
hybr_NOx_Fe = [light_off_hybrid.NOx_TP_Fe.val(1:end-1), ...
    fuel_hybrid.NOx_TP_Fe.val];

% Combine data from LO optimal and fuel optimal, conventional
conv_NOx_Cu = [light_off_conventional.NOx_TP_Cu.val(1:end-1), ...
    fuel_conventional.NOx_TP_Cu.val];
conv_NOx_Fe = [light_off_conventional.NOx_TP_Fe.val(1:end-1), ...
    fuel_conventional.NOx_TP_Fe.val];

% Combine data from LO optimal and fuel optimal, fuel consumption
conv_W_f = [light_off_conventional.W_f.val(1:end-1), ...
    fuel_conventional.W_f.val];
hybr_W_f = [light_off_hybrid.W_f.val(1:end-1), ...
    fuel_hybrid.W_f.val];

% Display data
fprintf('Conventional Cu NOx:   %.2f g\n', trapz(time_conv, conv_NOx_Cu));
fprintf('Conventional Fe NOx:   %.2f g\n', trapz(time_conv, conv_NOx_Fe));
fprintf('Hybrid Cu NOx:         %.2f g\n', trapz(time_hybr, hybr_NOx_Cu));
fprintf('Hybrid Fe NOx:         %.2f g\n', trapz(time_hybr, hybr_NOx_Fe));
fprintf('Hybrid Cu-opt, Cu NOx: %.2f g\n', trapz(time_NOx_Cu_optimal, NOx_Cu_optimal.NOx_TP_Cu.val));
fprintf('Hybrid Cu-opt, Fe NOx: %.2f g\n', trapz(time_NOx_Cu_optimal, NOx_Cu_optimal.NOx_TP_Fe.val));
fprintf('Hybrid Fe-opt, Cu NOx: %.2f g\n', trapz(time_NOx_Fe_optimal, NOx_Fe_optimal.NOx_TP_Cu.val));
fprintf('Hybrid Fe-opt, Fe NOx: %.2f g\n\n', trapz(time_NOx_Fe_optimal, NOx_Fe_optimal.NOx_TP_Fe.val));

fprintf('Conventional energy:   %.2f MJ\n', trapz(time_conv, conv_W_f).*param.ld2.q_HV.*1e-6);
fprintf('Hybrid Energy:         %.2f MJ\n', trapz(time_hybr, hybr_W_f).*param.ld2.q_HV.*1e-6);
fprintf('Hybrid Cu-opt, energy: %.2f MJ\n', trapz(time_NOx_Cu_optimal, NOx_Cu_optimal.W_f.val).*param.ld2.q_HV.*1e-6);
fprintf('Hybrid Fe-opt, energy: %.2f MJ\n\n', trapz(time_NOx_Fe_optimal, NOx_Fe_optimal.W_f.val).*param.ld2.q_HV.*1e-6);

fprintf('Conventional fuel:     %.0f g\n', trapz(time_conv, conv_W_f).*1e3);
fprintf('Hybrid fuel:           %.0f g\n', trapz(time_hybr, hybr_W_f).*1e3);
fprintf('Hybrid Cu-opt, fuel:   %.0f g\n', trapz(time_NOx_Cu_optimal, NOx_Cu_optimal.W_f.val).*1e3);
fprintf('Hybrid Fe-opt, fuel:   %.0f g\n\n', trapz(time_NOx_Fe_optimal, NOx_Fe_optimal.W_f.val).*1e3);

%% PLOT FOR COMPARING

figure(); hold on;
plot(time_hybr, xvec_hybr(:,15)-273, 'Color', [0.2 0.4 0.8], 'LineWidth', 2);
plot(time_conv, xvec_conv(:,14)-273, 'Color', [0.8 0.1 0.1], 'LineWidth', 2);
plot(time_NOx_Cu_optimal, xvec_NOx_Cu_optimal(:,15)-273, 'Color', [0.2 0.7 0.2], 'LineWidth', 2)
plot(time_NOx_Fe_optimal, xvec_NOx_Fe_optimal(:,15)-273, 'Color', [0.1 0.1 0.1], 'LineWidth', 2)

ylabel('SCR3 temperature (°C)');
xlabel('Time (s)');
yyaxis right;
plot(time_hybr, NOx_Cu1, ':', 'Color', [0.2 0.4 0.8], 'LineWidth', 1.5);
plot(time_conv, NOx_Cu2, ':', 'Color', [0.8 0.1 0.1], 'LineWidth', 1.5);
plot(time_NOx_Cu_optimal, NOx_Cu3, ':', 'Color', [0.2 0.7 0.2], 'LineWidth', 1.5);

plot(time_hybr, NOx_Fe1, '--', 'Color', [0.2 0.4 0.8], 'LineWidth', 1.5);
plot(time_conv, NOx_Fe2, '--', 'Color', [0.8 0.1 0.1], 'LineWidth', 1.5);
plot(time_NOx_Fe_optimal, NOx_Fe4, '--', 'Color', [0.1 0.1 0.1], 'LineWidth', 1.5);

ylabel('NOx (g)');
legend('Hybrid temp.', 'Conv. temp.', 'Hybrid Cu-NOx-opt. temp.', 'Hybrid Fe-NOx-opt. temp.', ...
    'Hybrid Cu NOx', 'Conv. Cu NOx', 'Hybrid Cu-NOx-opt. Cu NOx', ...
    'Hybrid Fe NOx', 'Conv. Fe NOx', 'Hybrid Fe-NOx-opt. Fe NOx', ...
    'Location','SouthEast');
title('Optimal SCR light-off');

%% OPTIMAL CONTROL FOR  NOx Cu PLOTS

figure(); hold on;
subplot(211); hold on;

plot(time_NOx_Cu_optimal, NOx_Cu_optimal.M_ice.val, 'Color', [0.2 0.7 0.2], 'LineWidth', 2);
plot(time_NOx_Cu_optimal, NOx_Cu_optimal.M_em.val, 'Color', [0.2 0.4 0.8], 'LineWidth', 2);

ylim([-2000, 2400]);
ylabel('Torque (Nm)');
title('Optimal control for minimum NOx');
yyaxis right
plot(time_NOx_Cu_optimal, NOx_Cu_optimal.deNOx_Cu.val, ':', 'LineWidth', 2);
ylabel('Cu-Z deNOx performance [0,1]');
xlabel('Time (s)');
ylim([0.9, 1]);

subplot(212);
plot(time_NOx_Cu_optimal, xvec_NOx_Cu_optimal(:,15)-273, 'Color', [0.2 0.7 0.2], 'LineWidth', 2);
ylabel('SCR3 temperature (°C)');
yyaxis right
plot(time_NOx_Cu_optimal, NOx_Cu_optimal.u_wg.val, ':', 'LineWidth', 2);
ylabel('Wastegate [0,1]');
xlabel('Time (s)');

%% OPTIMAL CONTROL FOR  NOx Fe PLOTS

figure(); hold on;
subplot(311)
plot(NOx_Fe_optimal.t_vec.val, NOx_Fe_optimal.SOC.val,'LineWidth',1.5);
title('Min. NOx with Fe-Zeolite');
ylim([0.25 0.5]);
line([0 1200],[0.375, 0.375]);
ylabel('SOC');
yyaxis right
plot(NOx_Fe_optimal.t_vec.val, NOx_Fe_optimal.u_f.val,'LineWidth',1.5)
ylabel('u_f (mg/cycle)');

subplot(312)
plot(NOx_Fe_optimal.t_vec.val, NOx_Fe_optimal.T_atw_c.val,'r','LineWidth',1.5);
ylabel('EO exh. temp.');
ylim([0, 650]);
yyaxis right
plot(NOx_Fe_optimal.t_vec.val, NOx_Fe_optimal.T_SCR3.val-273,'k','LineWidth',1.5)
ylabel('T_{SCR3} (\circC)');
subplot(313)
plot(NOx_Fe_optimal.t_vec.val, NOx_Fe_optimal.u_wg.val,'Color', [0.2 0.7 0.2],'LineWidth',1.5);
xlabel('Time (s)');
ylabel('Wastegate position');


%% PLOT ENERGY

E = zeros(length(NOx_Cu_optimal.t_vec.val),1);
for i = 2:length(NOx_Cu_optimal.t_vec.val(1:131))
    E(i) = trapz(NOx_Cu_optimal.t_vec.val(1:i), NOx_Cu_optimal.W_f.val(1:i)).*param.ld2.q_HV - ...
 	(NOx_Cu_optimal.x_vec.val(i,7)-NOx_Cu_optimal.x_vec.val(1,7)).* ...
    param.batpack.Q.*param.batpack.Vnom./param.batpack.series_cells;
end
plot(NOx_Cu_optimal.t_vec.val, E)

%% PLOT HYBRID STANDARD

P_out = 800*1100*pi/30;
P_in = light_off_hybrid.W_f.val.*param.ld2.q_HV + light_off_hybrid.P_b.val;
eta = P_out./P_in;
plot(light_off_hybrid.t_vec.val,eta);
yyaxis right
plot(light_off_hybrid.t_vec.val,light_off_hybrid.W_tw.val,'LineWidth',1.5)

figure(); hold on;
plot(light_off_hybrid.t_vec.val, light_off_hybrid.T_atw_c.val - light_off_hybrid.T_SCR3.val+273)

figure(); hold on;

subplot(211)
plot(light_off_hybrid.t_vec.val, light_off_hybrid.W_tw.val,'LineWidth',1.5);
ylabel('W_{tw} (kg/s)');
yyaxis right
plot(light_off_hybrid.t_vec.val,light_off_hybrid.u_wg.val,'LineWidth',1.5);
ylabel('Temperature after turb. and WG (\circC)');
xlabel('Time (s)');

subplot(212); hold on;
plot(light_off_hybrid.t_vec.val, light_off_hybrid.T_atw_c.val,'LineWidth',1.5);
plot(light_off_hybrid.t_vec.val, light_off_hybrid.T_SCR3.val-273,'LineWidth',1.5);
ylabel('Temperature (\circC)');
xlabel('Time (s)');


%% PRESENTATION KONVENTIONELL
figure(); hold on;
subplot(211);
plot(light_off_conventional.t_vec.val, light_off_conventional.u_wg.val, 'LineWidth', 2);
ylabel('Wastegate position [0, 1]');
ylim([-0.05 1.05]);
yyaxis right
plot(light_off_conventional.t_vec.val, light_off_conventional.u_f.val, 'LineWidth', 2);
ylabel('Massflow (kg/s)');
legend('WG', 'Massflow','Location', 'SE');
grid on;

subplot(212); hold on;
plot(light_off_conventional.t_vec.val, light_off_conventional.T_atw_c.val, 'LineWidth', 2);
plot(light_off_conventional.t_vec.val, light_off_conventional.T_SCR3.val-273, 'LineWidth', 2)
legend('Engine-Out','SCR');
ylabel('Temperature (\circC)');
xlabel('Time (s)');
grid on;
%% PRESENTATION HYBRID
figure(); hold on;
subplot(211);
plot(light_off_hybrid.t_vec.val, light_off_hybrid.u_wg.val, 'LineWidth', 2);
ylabel('Wastegate position [0, 1]');
ylim([-0.05 1.05]);
yyaxis right
plot(light_off_hybrid.t_vec.val, light_off_hybrid.W_tw.val, 'LineWidth', 2);
ylabel('Massflow (kg/s)');
legend('WG', 'Massflow','Location', 'SE');
grid on;

subplot(212); hold on;
plot(light_off_hybrid.t_vec.val, light_off_hybrid.T_atw_c.val, 'LineWidth', 2);
plot(light_off_hybrid.t_vec.val, light_off_hybrid.T_SCR3.val-273, 'LineWidth', 2)
legend('Engine-Out','SCR');
ylabel('Temperature (\circC)');
xlabel('Time (s)');
grid on;

%% HYBRID PRESENTATION

figure(); hold on;
subplot(211); hold on;
plot(light_off_hybrid.t_vec.val, light_off_hybrid.M_em.val, 'LineWidth', 2);
plot(light_off_hybrid.t_vec.val, light_off_hybrid.M_ice.val, '-.', 'LineWidth', 2);
plot(light_off_hybrid.t_vec.val, light_off_hybrid.M_em.val + light_off_hybrid.M_ice.val, '-', 'LineWidth', 1);
grid on;

ylabel('Torque (Nm)');
title('Optimal control for minimum NOx');
ylim([-2000 2500]);
xlim([0 180]);
legend('EM', 'ICE', 'EM+ICE');

subplot(212);
plot(light_off_hybrid.t_vec.val, light_off_hybrid.u_wg.val, 'LineWidth', 2);
ylabel('Wastegate position [0, 1]');
xlim([0 180]);
ylim([-0.05 1.05]);
grid on;
%------------- END OF CODE --------------


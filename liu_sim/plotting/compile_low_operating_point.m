% NAME: compile_low_operating_point
%
% PURPOSE:  Load and combine runs made with fuel and light-off time as
%           objective. Compare them with runs made with NOx Cu-Zeolite and 
%           Fe-Zeolite optimal. Plot interesting things. Steady state at
%           550 rpm and 250 Nm
%
% OTHER FILES REQUIRED:
%   .m files:
%       createParam__hev_eats_ss
%
%   .mat files:
%       ld2_eng_map
%       kallstart_550rpm_250nm_EATS_ld2_NOx_0_1114_K300d4
%       kallstart_550rpm_250nm_EATS_HEV_0_651_K100d4
%       kallstart_550rpm_250nm_EATS_HEV_651_1114_K1d4_fuel
%       kallstart_550rpm_250nm_EATS_HEV_0_1114_K10d4_NOx_TP_Cu
%       kallstart_EATS_HEV_0_1114_K300d4_TP_Fe
%   
% SUBFUNCTIONS:
%   none
%
% Author: Hampus Andersson and Fredrik Andersson
% May 2018; Last revision: 22-May-2018

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

% CONVENTIONAL
conventional = load_run('kallstart_550rpm_250nm_EATS_ld2_NOx_0_1114_K300d4', 0);

% HYBRID STANDARD
hybrid1 = load_run('kallstart_550rpm_250nm_EATS_HEV_0_651_K100d4', 0);
hybrid2 = load_run('kallstart_550rpm_250nm_EATS_HEV_651_1114_K1d4_fuel', 0);

% Cu-Zeolite OPTIMAL
hybrid_Cu = load_run('kallstart_550rpm_250nm_EATS_HEV_0_1114_K10d4_NOx_TP_Cu', 0);

% Fe-Zeolite OPTIMAL
hybrid_Fe = load_run('kallstart_EATS_HEV_0_1114_K300d4_TP_Fe', 0);

%% COMBINE DATA FROM RUNS

% Time
time_conv = conventional.t_vec.val;
time_hybr = [hybrid1.t_vec.val(1:end-1), ...
    hybrid2.t_vec.val + hybrid1.t_vec.val(end)];
time_Cu = hybrid_Cu.t_vec.val;
time_Fe = hybrid_Fe.t_vec.val;

% x-vec
x_vec_conv = conventional.x_vec.val;
x_vec_hybr = [hybrid1.x_vec.val(1:end-1,:); ...
    hybrid2.x_vec.val];
x_vec_Cu = hybrid_Cu.x_vec.val;
x_vec_Fe = hybrid_Fe.x_vec.val;

% NOx Cu and Fe
NOx_Cu_instant_conv = conventional.NOx_TP_Cu.val;
NOx_Fe_instant_conv = conventional.NOx_TP_Fe.val;
NOx_Cu_instant_hybr = [hybrid1.NOx_TP_Cu.val(1:end-1), ...
    hybrid2.NOx_TP_Cu.val];
NOx_Fe_instant_hybr = [hybrid1.NOx_TP_Fe.val(1:end-1), ...
    hybrid2.NOx_TP_Fe.val];
NOx_Cu_instant_Cu_opt = hybrid_Cu.NOx_TP_Cu.val;
NOx_Fe_instant_Cu_opt = hybrid_Cu.NOx_TP_Fe.val;
NOx_Cu_instant_Fe_opt = hybrid_Fe.NOx_TP_Cu.val;
NOx_Fe_instant_Fe_opt = hybrid_Fe.NOx_TP_Fe.val;

% Integrate NOx over run
NOx_Cu_conv = zeros(length(time_conv),1);
NOx_Fe_conv = zeros(length(time_conv),1);
NOx_Cu_hybr = zeros(length(time_hybr),1);
NOx_Fe_hybr = zeros(length(time_hybr),1);
NOx_Cu_Cu_opt = zeros(length(time_Cu),1);
NOx_Fe_Cu_opt = zeros(length(time_Cu),1);
NOx_Cu_Fe_opt = zeros(length(time_Fe),1);
NOx_Fe_Fe_opt = zeros(length(time_Fe),1);
for i = 2:length(time_conv)
    NOx_Cu_conv(i) = trapz(time_conv(1:i),NOx_Cu_instant_conv(1:i));
    NOx_Fe_conv(i) = trapz(time_conv(1:i),NOx_Fe_instant_conv(1:i));
end
for i = 2:length(time_hybr)
    NOx_Cu_hybr(i) = trapz(time_hybr(1:i),NOx_Cu_instant_hybr(1:i));
    NOx_Fe_hybr(i) = trapz(time_hybr(1:i),NOx_Fe_instant_hybr(1:i));
end
for i = 2:length(time_Cu)
    NOx_Cu_Cu_opt(i) = trapz(time_Cu(1:i), NOx_Cu_instant_Cu_opt(1:i)); 
    NOx_Fe_Cu_opt(i) = trapz(time_Cu(1:i), NOx_Fe_instant_Cu_opt(1:i));
end
for i = 2:length(time_Fe)
    NOx_Cu_Fe_opt(i) = trapz(time_Fe(1:i), NOx_Cu_instant_Fe_opt(1:i)); 
    NOx_Fe_Fe_opt(i) = trapz(time_Fe(1:i), NOx_Fe_instant_Fe_opt(1:i));
end

%% PRINT DATA FOR COMPARISON

% Combine data from LO optimal and fuel optimal, conventional
conv_NOx_Cu = conventional.NOx_TP_Cu.val;
conv_NOx_Fe = conventional.NOx_TP_Fe.val;
conv_W_f = conventional.W_f.val;

% Combine data from LO optimal and fuel optimal, hybrid
hybr_NOx_Cu = [hybrid1.NOx_TP_Cu.val(1:end-1), ...
    hybrid2.NOx_TP_Cu.val];
hybr_NOx_Fe = [hybrid1.NOx_TP_Fe.val(1:end-1), ...
    hybrid2.NOx_TP_Fe.val];
hybr_W_f = [hybrid1.W_f.val(1:end-1), ...
    hybrid2.W_f.val];

% Display data
fprintf('Conventional Cu NOx:   %.2f g\n',trapz(time_conv, conv_NOx_Cu));
fprintf('Conventional Fe NOx:   %.2f g\n',trapz(time_conv, conv_NOx_Fe));
fprintf('Hybrid Cu NOx:         %.2f g\n',trapz(time_hybr, hybr_NOx_Cu));
fprintf('Hybrid Fe NOx:         %.2f g\n',trapz(time_hybr, hybr_NOx_Fe));
fprintf('Hybrid Cu-opt, Cu NOx: %.2f g\n',trapz(time_Cu, hybrid_Cu.NOx_TP_Cu.val));
fprintf('Hybrid Cu-opt, Fe NOx: %.2f g\n',trapz(time_Cu, hybrid_Cu.NOx_TP_Fe.val));
fprintf('Hybrid Fe-opt, Cu NOx: %.2f g\n',trapz(time_Fe, hybrid_Fe.NOx_TP_Cu.val));
fprintf('Hybrid Fe-opt, Fe NOx: %.2f g\n\n',trapz(time_Fe, hybrid_Fe.NOx_TP_Fe.val));

fprintf('Conventional energy:   %.2f MJ\n', trapz(time_conv, conv_W_f).*param.ld2.q_HV.*1e-6);
fprintf('Hybrid energy:         %.2f MJ\n', trapz(time_hybr, hybr_W_f).*param.ld2.q_HV.*1e-6);
fprintf('Hybrid Cu-opt, energy: %.2f MJ\n', trapz(time_Cu, hybrid_Cu.W_f.val).*param.ld2.q_HV.*1e-6);
fprintf('Hybrid Fe-opt, energy: %.2f MJ\n\n', trapz(time_Fe, hybrid_Fe.W_f.val).*param.ld2.q_HV.*1e-6);

fprintf('Conventional fuel:    %.0f g\n', trapz(time_conv, conv_W_f).*1e3);
fprintf('Hybrid fuel:          %.0f g\n', trapz(time_hybr, hybr_W_f).*1e3);
fprintf('Hybrid Cu-opt, fuel:  %.0f g\n', trapz(time_Cu, hybrid_Cu.W_f.val).*1e3);
fprintf('Hybrid Fe-opt, fuel:  %.0f g\n\n', trapz(time_Fe, hybrid_Fe.W_f.val).*1e3);

%% COMPARABLE PLOTS FOR LIGHT-OFF BEHAVIOR

figure(); hold on;
plot(time_conv, x_vec_conv(:,14)-273, 'Color', [0.8 0.1 0.1], 'LineWidth', 2);
plot(time_hybr, x_vec_hybr(:,15)-273, 'Color', [0.2 0.4 0.8], 'LineWidth', 2);
plot(time_Cu, x_vec_Cu(:,15)-273, 'Color', [0.2 0.7 0.2], 'LineWidth', 2)
plot(time_Fe, x_vec_Fe(:,15)-273, 'Color', [0.9 0.8 0.3], 'LineWidth', 2)

ylabel('SCR3 temperature (°C)');
xlabel('Time (s)');
yyaxis right;
plot(time_conv, NOx_Cu_conv, ':', 'Color', [0.8 0.1 0.1], 'LineWidth', 1.5);
plot(time_hybr, NOx_Cu_hybr, ':', 'Color', [0.2 0.4 0.8], 'LineWidth', 1.5);
plot(time_Cu, NOx_Cu_Cu_opt, ':', 'Color', [0.2 0.7 0.2], 'LineWidth', 1.5);
%plot(time4, NOx_Cu4, ':', 'Color', [0.9 0.8 0.3], 'LineWidth', 1.5);

plot(time_conv, NOx_Fe_conv, '--', 'Color', [0.8 0.1 0.1], 'LineWidth', 1.5);
plot(time_hybr, NOx_Fe_hybr, '--', 'Color', [0.2 0.4 0.8], 'LineWidth', 1.5);
%plot(time3, NOx_Fe3, '--', 'Color', [0.2 0.7 0.2], 'LineWidth', 1.5);
plot(time_Fe, NOx_Fe_Fe_opt, '--', 'Color', [0.9 0.8 0.3], 'LineWidth', 1.5);

ylabel('NOx (g)');
legend('Conv. temp.', 'Hybr. temp.', 'Hybr. Cu-NOx-opt. temp.', 'Hybr. Fe-NOx-opt. temp.', ...
    'Conv. Cu NOx', 'Hybr. Cu NOx', 'Hybr. Cu-NOx-opt. Cu NOx', ... 'Hybr. Fe-NOx-opt. Cu_NOx', ...
    'Conv. Fe NOx', 'Hybr. Fe NOx', ...'Hybr. Cu-NOx-opt. Fe NOx', 
    'Hybr. Fe-NOx-opt. Fe NOx', ...
    'Location','SouthEast');
title('Cold start control');
grid on

%% OPTIMAL CONTROL FOR  NOx plots

figure(); hold on;
subplot(211);
plot(time_Cu, x_vec_Cu(:,15)-273, 'Color', [0.2 0.7 0.2], 'LineWidth', 2);
ylabel('SCR3 temperature (°C)');
title('Optimal control for minimum NOx');
yyaxis right
plot(time_Cu, hybrid_Cu.u_f.val, ':', 'LineWidth', 2);
ylabel('Fuel injected (mg/cycle)');
xlabel('Time (s)');

subplot(212);
plot(time_Cu, NOx_Cu_Cu_opt, 'Color', [0.2 0.7 0.2], 'LineWidth', 2);
ylabel('NOx (g)');
yyaxis right
plot(time_Cu, hybrid_Cu.deNOx_Cu.val, ':', 'LineWidth', 2);
ylabel('Cu-Z deNOx performance [0,1]');
xlabel('Time (s)');

%% OPTIMAL CONTROL FOR  NOx plots 2

figure(); hold on;
subplot(211); hold on;
plot(time_Cu, hybrid_Cu.M_ice.val, 'LineWidth', 2);
plot(time_Cu, hybrid_Cu.M_em.val, 'LineWidth', 2);
plot(time_Cu, hybrid_Cu.M_em.val+hybrid_Cu.M_ice.val, 'LineWidth', 2);


ylabel('Torque (Nm)');
title('Optimal control for minimum NOx');
ylim([-1500 1500]);
yyaxis right
plot(time_Cu, hybrid_Cu.deNOx_Cu.val, ':', 'LineWidth', 2);
ylabel('deNOx performance [0, 1]');
xlabel('Time (s)');
ylim([0.7 0.95]);
xlim([0 1114]);

subplot(212);
plot(time_Cu, hybrid_Cu.T_SCR3.val-273, 'Color', [0.2 0.7 0.2], 'LineWidth', 2);
ylabel('SCR temperature (\circC)');
yyaxis right
plot(time_Cu, NOx_Cu_Cu_opt, ':', 'LineWidth', 2);
ylabel('NOx (g)');
xlabel('Time (s)');
xlim([0 1114]);

%% HYBRID LIGHT-OFF

figure(); hold on;
title('Light-off Hybrid');
xlabel('Time (s)');
plot(hybrid1.t_vec.val, hybrid1.SOC.val.*100, ...'Color', [0.8 0.1 0.1], 
    'LineWidth', 2);
ylabel('State of charge (%)');
yyaxis right
plot(hybrid1.t_vec.val, hybrid1.u_wg.val, ':', 'LineWidth', 2);
ylabel('Wastegate position [0, 1]');


%% CONVENTIONAL LIGHT-OFF

figure(); hold on;
title('Light-off Conventional');
xlabel('Time (s)');
plot(time_conv, x_vec_conv(:,14)-273, 'Color', [0.8 0.1 0.1], 'LineWidth', 2);
ylabel('SCR3 temperature (°C)');
yyaxis right
plot(time_conv, conventional.N_t.val, ':', 'LineWidth', 2);
ylabel('krpm');

%% PRESENTATION KONVENTIONELL
figure(); hold on;
subplot(211);
plot(conventional.t_vec.val, conventional.u_wg.val, 'LineWidth', 2);
ylabel('Wastegate position [0, 1]');
ylim([-0.05 1.05]);
%yyaxis right
%plot(conventional.t_vec.val, conventional.x_vec.val, 'LineWidth', 2);
%ylabel('Massflow (kg/s)');
%legend('WG', 'Massflow','Location', 'NE');
grid on;
xlim([0 1114]);

subplot(212); hold on;
plot(conventional.t_vec.val, conventional.T_atw_c.val, 'LineWidth', 2);
plot(conventional.t_vec.val, conventional.T_SCR3.val-273, 'LineWidth', 2)
legend('Engine-Out','SCR','Location', 'SE');
ylabel('Temperature (\circC)');
xlabel('Time (s)');
xlim([0 1114]);
grid on;


%% HYBRID PRESENTATION

figure(); hold on;
subplot(211); hold on;
plot(hybrid1.t_vec.val, hybrid1.M_em.val, 'LineWidth', 2);
plot(hybrid1.t_vec.val, hybrid1.M_ice.val, '-.', 'LineWidth', 2);
plot(hybrid1.t_vec.val, hybrid1.M_em.val + hybrid1.M_ice.val, '-', 'LineWidth', 1);
grid on;

ylabel('Torque (Nm)');
title('Optimal control for minimum NOx');
ylim([-1400 1400]);
xlim([0 648]);

subplot(212);
plot(hybrid1.t_vec.val, hybrid1.u_wg.val, 'LineWidth', 2);
ylabel('Wastegate position [0, 1]');
xlim([0 648]);
ylim([-0.05 1.05]);
grid on;

%------------- END OF CODE --------------

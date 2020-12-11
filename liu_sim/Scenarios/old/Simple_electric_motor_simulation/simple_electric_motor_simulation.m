%% Setup simulation
clear all;
close all;
clc;

% Control indices, for reabability
[u_f, u_thr, u_wg, M_em] = enum(4);

% Load engine parameters. Ignore warnings
%load('liu_diesel_2_params.mat');
main_hev;

% setup engine model variable
ld2_HEV = @(X, U, N_ice) hybridPowertrain(X, U, N_ice, param);

% setup simulator
opts = odeset('RelTol', 1e-9, 'AbsTol', 1e-9);
simulate = @(model, X0, U, N_ice, tf) ...
             ode15s(@(t,x) model(x, U, N_ice), [0, tf], X0, opts);

%% Simulate a step in fuel injection at ~1200 rpm
N_ice_des = 1200;

% Load steady-state setpoint from the LD2 engine map
load('ld2_eng_map.mat')

% Find the steady-state operating point closest to the desired speed in the engine
% map
[~, N_index] = min(abs( engine_map.rpm_range - N_ice_des ));
N_ice_sim = engine_map.rpm_range(N_index);

% Start the engine from the lowest calibrated torque, at the desired speed
X0 = [cell2mat(engine_map.state_calibration(1, N_index)); 0;param.em.batpack.SOC];
U0 = [cell2mat(engine_map.control_calibration(1, N_index)); 0];

% Add step to control signal
rel_step = 120;
U = [U0(u_f) + rel_step; U0(u_thr); U0(u_wg); 500]; %Tillagd

% Simulate step
tf = 100; % end time [s]
[t_sim, x_sim] = simulate(ld2_HEV, X0, U, N_ice_sim, tf);

%% Calculate internal signals based on simulation results
signals = internal_signals_HEV(@(x, u)ld2_HEV(x, u, N_ice_sim), ...
                                  t_sim, ...
                                  x_sim, ...
                                  repmat(U', length(t_sim), ...
                                  1));

%% plot simulation signals
close all
ld2_plots(signals, engine_map);
NOx_plots(signals);
em_plots(signals);




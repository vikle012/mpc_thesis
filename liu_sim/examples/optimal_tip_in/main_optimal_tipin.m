%% Optimal tip-in example
close all
clear all
clc
% Load engine parameters. Ignore warnings
load('ld2_pwl_param.mat');

% Desired engine speed
N_ice_des = 1200;

% Load steady-state setpoint from the LD2 engine map
load('ld2_eng_map.mat')

% Find the steady-state operating point closest to the desired speed in the 
% engine map
[~, N_index] = min(abs( engine_map.rpm_range - N_ice_des ));
N_ice_opt = engine_map.rpm_range(N_index);

% Start the engine from the lowest calibrated torque, at the desired speed
x0 = cell2mat(engine_map.state_calibration(1, N_index));
u0 = cell2mat(engine_map.control_calibration(1, N_index));

% Terminate the optimzation at the highest load in the map
xT = cell2mat(engine_map.state_calibration(end, N_index));
uT = cell2mat(engine_map.control_calibration(end, N_index));

% Formulate OCP
ocp = optimal_tipin_ocp(x0, u0, xT, uT, N_ice_opt, pwl_param);

% Simulate the system to get inital guess
v0 = optimal_tipin_initial_guess(x0, u0, N_ice_opt, pwl_param);

% Number of discretization segments
K = 25;

nlpOpts = struct;
nlpOpts.ipopt.acceptable_tol = 1e-6;
nlpOpts.ipopt.tol = 1e-7;
opts = {'collocation_mode', 'normal', ...     'normal' or 'rigid'
        'no_collocation_points', 5, ...        >= 1
        'collocation_points', 'legendre', ... 'legendre' or 'radau'
        'collocation_intervals', 1, ...         >= 1
        'nlp_opts', nlpOpts, ...
        };
[ocp_sol, v0] = yopSolve(ocp, v0, 'collocation', K, opts);


% Plot results
t_opt = ocp_sol.getTimeVec(1);
x_opt = ocp_sol.getStateMat(1);
u_opt = ocp_sol.getControlMat(1);
u_opt(end+1,:) = u_opt(end,:);

ld2 = @(x, u) LD2_pwl(x, u, N_ice_opt, pwl_param);
opt_signals = engine_internal_signals(ld2, t_opt, x_opt, u_opt);

LD2_fig_init(1);
example_plots(opt_signals, engine_map)

% Validate results by simulating the control input
% [t_ip, u_ip] = prepare_interpolation_data(t_opt, u_opt);
% sim_model = @(t,x) validation_model_wrapper(t, x, t_ip, u_ip, ld2);
% 
% opts = odeset('RelTol', 1e-9, 'AbsTol', 1e-9);
% [t_sim, x_sim] = ode15s(sim_model, [0, t_opt(end)], x_opt(1,:), opts);
% u_sim = interp1(t_ip, u_ip, t_sim);
% 
% sim_signals = engine_internal_signals(ld2, t_sim, x_sim, u_sim);
% 
% LD2_fig_init(1);
% example_plots(sim_signals, engine_map)


























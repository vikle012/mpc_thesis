close all;
clc;
fprintf('Parallel HEV!\n\n');

%% INITIALIZE
load('WHTC_data.mat')
load('ld2_eng_map.mat')

% only create 'param' if it does not exist
if ~exist('param', 'var')
    param = struct;
    param = init_hev_pwl(param);
    speed_scaled = speed .* 19 + 500; % Speed from [0, 100] to [500, 2400] rpm
    param = create_WHTC(time, speed_scaled, torque, engine_map, param);
end

%%

N_init = 500;   % Initial engine speed
M_init = 0;     % Initial engine torque
N_term = 500;   % Terminal engine speed
M_term = 0;     % Terminal engine torqur

% retrieve values for steady state operation for given torque and speed
[~, N_index_init] = min(abs( engine_map.rpm_range - N_init ));
[~, M_index_init] = min(abs( engine_map.torque_grid(:, N_index_init) - M_init ));
[~, N_index_term] = min(abs( engine_map.rpm_range - N_term ));
[~, M_index_term] = min(abs( engine_map.torque_grid(:, N_index_term) - M_term ));
N_init = engine_map.rpm_range(N_index_init);
M_init = engine_map.torque_grid(M_index_init, N_index_init);
N_term = engine_map.rpm_range(N_index_term);
M_term = engine_map.torque_grid(M_index_term, N_index_term);

%% INITIAL
% states
x0 = [engine_map.state_calibration{M_index_init, N_index_init}, ... % ld2 x0 min
    engine_map.state_calibration{M_index_init, N_index_init}; ... % ld2 x0 max
    1e-6,	1e-6; ...      % NOx min, max (bad initial guess if = 0)
    500,	500; ...        % Engine speed min, max [rpm]
    0.375,	0.375 ...       % SOC min, max
    ];
% controls
u0 = [engine_map.control_calibration{M_index_init, N_index_init}, ... % ld2 u0 min
    engine_map.control_calibration{M_index_init, N_index_init}; ...   % ld2 u0 max
    -1500,  1500 ...       % M_em min, max
    ];

%% TERMINAL
% states
xT = [param.ld2.constr.x_min(1:6), ... % ld2 x0 min
      param.ld2.constr.x_max(1:6);  ...   % ld2 x0 max
%       1e-9,  10; ...            % NOx min, max
%       500,    2400; ...         % Engine speed min, max
       0.375,  0.375 ...         % SOC min, max (charge sustaining if min above x0(7), leave max at SOCmax)
    ];

% controls
uT = [param.ld2.constr.x_min(8:11), ... % ld2 u0 min
    param.ld2.constr.x_max(8:11) ...   % ld2 uu0 max
    ];


%% SOLVE FOR OPTIMAL SOLUTION

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tf = 8;     % End time [s] Hittar optimum för 8s, max iterations på 9
K = tf;     % Segments (should equal cycle duration in sec)
d = 5;      % Collocation points
N = 1;      % Collocation intervals per segment (normally = 1)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% creating OCP
ocp             = hev_tipin_ocp(x0, u0, xT, uT, tf, param);
% initial guess
v0              = hev_tipin_initial_guess(x0, u0, tf, param);
% creating NLP and solving it
[ocp_sol, opt_v0]   = yopSolve_WHTC(ocp, v0, K, d, N);

%% PLOTTING
t_opt           = ocp_sol.getTimeVec(1);
x_opt           = ocp_sol.getStateMat(1);
u_opt           = ocp_sol.getControlMat(1);
u_opt(end+1, :) = u_opt(end, :);

% simulating optimal solution to retrieve optimal signals
hybrid = @(x, u, t) hybridPowertrain_pwl(x, u, t, param);
opt_signals = internal_signals_HEV(hybrid, t_opt, x_opt, u_opt);

% plotting functions
%ld2_plots(opt_signals, engine_map);
%NOx_plots(opt_signals);
%em_plots(opt_signals);
hev_plots(opt_signals);

% initial guess performance
hold off; figure();
plot(v0.T, v0.X(:,6), 'k', 'LineWidth', 1); hold on;
plot(t_opt, x_opt(:,6), 'r--');
plot(v0.T, v0.X(:,5), 'k', 'LineWidth', 1)
plot(t_opt, x_opt(:,5), 'r--');
legend('Initial guess','Optimal');
title('Fuel injection W_f and engine speed N');

% torque curve following
hold off; figure();
plot(time(1:tf), full(param.lut_M_WHTC(time(1:tf))), 'k', 'LineWidth', 1); hold on;
plot(opt_signals.t_vec.val, opt_signals.M_ice.val, 'r--');
plot(time(1:tf), torque(1:tf).*24, 'k--', 'LineWidth', 1);
legend('WHTC lut','M_{ice}','WHTC','Location','NorthWest');
title('Torque [Nm]');
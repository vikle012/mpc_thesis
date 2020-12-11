% NAME: MAIN__hev_eats_ss_free_N
%
% PURPOSE:  Diesel electric hybrid with a 450 hp diesel engine wihtout EGR
%           or VGT and a 210 hp electric motor in a parallel configuration.
%           The model has an engine out NOx estimation. The hybrid
%           powertrain is optimally controlled for the WHTC test cycle
%           using direct collocation. Throttle is assumed fully open to
%           increase optimization stability.
%
% OTHER FILES REQUIRED:
%   .m files:
%       createParam__hev_ss_free_N
%       ocp__hev_ss_free_N
%   	initGuess__hev_ss_free_N
%       yopSolve_WHTC
%       PT_PWL__hev_ss_free_N
%       internalSignals__hev_ss_free_N
%       whtcPlots
%       ld2Plots
%       noxPlots
%       emPlots
%       hevPlots
%
%   .mat files:
%       ld2_eng_map
%
% SUBFUNCTIONS:
%   none
%
% Author: Hampus Andersson and Fredrik Andersson
% Mars 2018; Last revision: 20-April-2018

%------------- BEGIN CODE --------------

close all
%clc
tic

%% VARIABLES

% Steady state idling in these points in WHTC:
% 0 -> 45 -> 140 -> 220 -> 350 -> 650 -> 720 -> 898 -> 1170 -> 1800

t0      = 0;            % Start time [s]
tf      = 1000;            % End time   [s]

N_init  = 550;          % Initial engine speed for steady state values
M_init  = 250;            % Initial engine torque for steady state values
% Change M_dem in 'PT__hev_eats_ss.m' also
K       = 400;%1*(tf-t0);    % Collocation segments (normally 1 or 2 per second)
d       = 4;            % Collocation points (normally = 5)
N       = 1;            % Collocation intervals per segment (normally = 1)

%% INITIALIZE AND LOAD PARAMS

load('ld2_eng_map.mat', 'engine_map')

% Only create 'param' if it does not already exist to save time
if ~exist('param', 'var')
    regen_brake = 1000; % Nm
    param = createParam__hev_eats_ss_free_N(engine_map, regen_brake);
end

% Retrieve closest values for steady state operation for given torque and speed
[~, N_index_init] = min(abs( engine_map.rpm_range - N_init ));
[~, M_index_init] = min(abs( engine_map.torque_grid(:, N_index_init) - M_init ));
%M_index_init = M_index_init + 1; % Margin
%% INITIAL STATES AND CONTROLS

% States
x0 = [engine_map.state_calibration{M_index_init, N_index_init}, ... ld2 x0 min
    engine_map.state_calibration{M_index_init, N_index_init} .* 1; ... ld2 x0 max
    param.ld2.constr.x_min(5), param.ld2.constr.x_min(5); ... NOx min, max [kg]
    0.375, 0.375; ...        SOC min, max
    ones(param.EATS.DOC.n_seg,2).*param.EATS.T_DOC_0; ...
    ones(param.EATS.DPF.n_seg,2).*param.EATS.T_DPF_0; ...
    ones(param.EATS.SCR.n_seg,2).*param.EATS.T_SCR_0; ...
    ones(1,2).*param.EATS.T_silencer_0 ...
    ];
% Controls
u0 = [engine_map.control_calibration{M_index_init, N_index_init}(1), ... u_f min
    engine_map.control_calibration{M_index_init, N_index_init}(1) .* 1; ... u_f max
    1, 1; ...               Throttle min, max
    0, 0; ...               Wastegate min, max
    -100, 100; ...          M_em min, max
    550, 550 ...            N min, max
    ];

%% TERMINAL STATES AND CONTROLS

% states (( LEAVE FROM MIN TO MAX ))
xT = [param.ld2.constr.x_min(1), param.ld2.constr.x_max(1); ... p_cac [0.1, 5] bar
    param.ld2.constr.x_min(2), param.ld2.constr.x_max(2); ...   p_im [0.1, 5] bar
    param.ld2.constr.x_min(3), param.ld2.constr.x_max(3); ...   p_em [0.4, 5] bar
    param.ld2.constr.x_min(4), param.ld2.constr.x_max(4); ...   w_tc [~1, ~12] krpm
    param.ld2.constr.x_min(5), param.ld2.constr.x_max(5); ...   NOx [0, 1] kg
    0.375, 0.5; ...                                               SOC [0.25 0.5]
    param.ld2.constr.x_min(8:14), param.ld2.constr.x_max(8:14); ...
    473, param.ld2.constr.x_max(15); ...
    param.ld2.constr.x_min(16:19), param.ld2.constr.x_max(16:19) ...
    ];

% controls (( LEAVE FROM MIN TO MAX ))
uT = [param.ld2.constr.x_min(20), param.ld2.constr.x_max(20); ... w_f [0, 280]
    param.ld2.constr.x_min(21), param.ld2.constr.x_max(21); ...  u_th [0, 1]
    param.ld2.constr.x_min(22), param.ld2.constr.x_max(22); ...  u_wg [0, 1]
    param.ld2.constr.x_min(23), param.ld2.constr.x_max(23); ...  M_em [-1500, 1500]
    550, 1800; ... N [500, 2400]
    ];

%% SOLVE FOR OPTIMAL SOLUTION

% Creating OCP
[ocp, info.L, info.E] = ocp__hev_eats_ss_free_N(x0, u0, xT, uT, t0, tf, param);
% Initial guess
v0 = initGuess__hev_eats_ss_free_N(x0, u0, t0, tf, param);
% test_check_v0;
% Creating NLP and solving it
% Optimal solution is found when objective is within 'tol'.
nlpOpts.ipopt.constr_viol_tol   = 1e-3;
nlpOpts.ipopt.max_iter          = 30000;
nlpOpts.ipopt.tol               = 1e-6;

% If 'acceptable_iter' many iterations occur while not improving objective
% within 'acceptable_tol', an acceptable solutions is found.
nlpOpts.ipopt.acceptable_iter   = 100;
nlpOpts.ipopt.acceptable_tol    = 1e-5;

[ocp_sol, opt_v0] = yopSolve_WHTC(ocp, v0, K, d, N, nlpOpts);

%% Information .mat
info.t0 = t0;
info.tf = tf;
info.K  = K/(tf-t0);
info.d  = d;
info.N  = N;
info.constr_viol_tol    = nlpOpts.ipopt.constr_viol_tol;
info.max_iter           = nlpOpts.ipopt.max_iter;
info.acceptable_iter    = nlpOpts.ipopt.acceptable_iter;
info.acceptable_tol     = nlpOpts.ipopt.acceptable_tol;
info.tol                = nlpOpts.ipopt.tol;
info.obj_norm           = ocp.scaling.obj_norm;
info.opt_duration       = toc;
info.time_stamp         = ocp_sol.time_stamp;

%% PLOTTING
close all;

t_opt           = ocp_sol.getTimeVec(1) + t0;
x_opt           = ocp_sol.getStateMat(1);
u_opt           = ocp_sol.getControlMat(1);
u_opt(end+1, :) = u_opt(end, :);

% Simulating optimal solution to retrieve optimal signals
hev = @(x, u, t) PT_PWL__hev_eats_ss_free_N(x, u, t, param, t0);
opt_signals = internalSignals__hev_eats_ss_free_N(hev, t_opt, x_opt, u_opt);
opt_signals.info = info;

%%%%%%%%%%%%%%%%%%%%%%%%%
v02 = v0;
v02.X = [v0.X(:,1:5), zeros(length(v0.X(:,1)),1), v0.X(:,6:end)];
opt_signals.x_vec.val = [opt_signals.x_vec.val(:,1:5), zeros(length(opt_signals.x_vec.val),1), opt_signals.x_vec.val(:,6:end)];
%%%%%%%%%%%%%%%%%%%%%%%%%

% Plotting functions
%ld2Plots(opt_signals, engine_map);
%noxPlots(opt_signals, engine_map);
%emPlots(v02, opt_signals);
%hevPlots(opt_signals);
whtcPlots(v02, opt_signals, param)

% Printing comparable data
fprintf('\nWHTC from %.0f to %.0f seconds\n',t0,tf);
fprintf('Fuel consumption: %.2f ml/s\n', litrePerSecond(opt_signals).*1e3);
fprintf('SOC: %.1f to %.1f percent\n', ...
    opt_signals.SOC.val(1).*100, opt_signals.SOC.val(end).*100);
fprintf('Total NOx: %.4f g\n', opt_signals.NOx_tot.val(end));

%------------- END OF CODE --------------
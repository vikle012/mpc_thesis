% NAME: MAIN__ld2_nox_whtc
%
% PURPOSE:  Conventional 450 hp diesel engine wihtout EGR or VGT with out
%           of engine NOx estimation. Engine is optimally controlled for
%           the WHTC test cycle using direct collocation. Optimization time
%           is roughly 1:1 to real time on longer runs on a 2.8 GHz core.
%           Throttle is assumed fully open to increase optimization
%           stability.
%
% OTHER FILES REQUIRED:
%   .m files:
%       createParam__ld2_nox_whtc
%       ocp__ld2_nox_whtc
%   	initGuess__ld2_nox_whtc
%       yopSolve_WHTC
%       PT_PWL__ld2_nox_whtc
%       internalSignals__ld2_nox_whtc
%       whtcPlots
%       ld2Plots
%       noxPlots
%
%   .mat files:
%       ld2_eng_map
%
% SUBFUNCTIONS:
%   none
%
% Author: Hampus Andersson and Fredrik Andersson
% Mars 2018; Last revision: 15-Mars-2018

%------------- BEGIN CODE --------------

%clear all;
close all;
clear all;
tic;

%% VARIABLES

% Steady state idling in these points in WHTC:
% 0 -> 45 -> 140 -> 220 -> 350 -> 650 -> 720 -> 898 -> 1170 -> 1800

t0      = 0;            % Start time [s]
tf      = 2000;            % End time   [s] 

N_init  = 500;          % Initial engine speed for steady state values
M_init  = 0;            % Initial engine torque for steady state values

K       = floor(0.5*(tf-t0));    % Collocation segments (normally 1 or 2 per second)
d       = 4;            % Collocation points (normally = 5)
N       = 1;            % Collocation intervals per segment (normally = 1)

%% INITIALIZE AND LOAD PARAMS

load('ld2_eng_map.mat', 'engine_map')

% Only create 'param' if it does not already exist to save time
if ~exist('param', 'var')
    param = createParam__ld2_nox_EATS_whtc(engine_map);
end

% Retrieve values for steady state operation for given torque and speed
[~, N_index_init] = min(abs( engine_map.rpm_range - N_init ));
[~, M_index_init] = min(abs( engine_map.torque_grid(:, N_index_init) - M_init ));

%% INITIAL STATES AND CONTROLS

% States
x0 = [engine_map.state_calibration{M_index_init, N_index_init}, ... ld2 x0 min
    engine_map.state_calibration{M_index_init, N_index_init} .* 1.2; ... ld2 x0 max
    param.ld2.constr.x_min(5), param.ld2.constr.x_min(5); ... NOx min, max [g]
    N_init,	N_init; ...     Engine speed min, max [rpm]
    ones(param.EATS.DOC.n_seg,2).*param.EATS.T_DOC_0; ...
    ones(param.EATS.DPF.n_seg,2).*param.EATS.T_DPF_0; ...
    ones(param.EATS.SCR.n_seg,2).*param.EATS.T_SCR_0; ...
    ones(1,2).*param.EATS.T_silencer_0 ...
    ];

% Controls
u0 = [engine_map.control_calibration{M_index_init, N_index_init}(1), ... u_f min
    engine_map.control_calibration{M_index_init, N_index_init}(1) .* 1.2; ... u_f max
    1, 1; ...               Throttle min, max
    0, 0 ...               Wastegate min, max
    ];

%% TERMINAL STATES AND CONTROLS

% States (( LEAVE FROM MIN TO MAX ))
xT = [param.ld2.constr.x_min(1), param.ld2.constr.x_max(1); ... p_cac [0.1, 5] bar
    param.ld2.constr.x_min(2), param.ld2.constr.x_max(2); ...   p_im [0.1, 5] bar
    param.ld2.constr.x_min(3), param.ld2.constr.x_max(3); ...   p_em [0.4, 5] bar
    param.ld2.constr.x_min(4), param.ld2.constr.x_max(4); ...   w_tc [~1, ~12] krpm
    param.ld2.constr.x_min(5), param.ld2.constr.x_max(5); ...   NOx ?? kg
    param.ld2.constr.x_min(6), param.ld2.constr.x_max(6); ...   N [400, 2500] rpm.
    [param.ld2.constr.x_min(7:18), param.ld2.constr.x_max(7:18)] ...
    ];

% Controls (( LEAVE FROM MIN TO MAX ))
uT = [param.ld2.constr.x_min(7), param.ld2.constr.x_max(19); ... w_f [0, 280]
    param.ld2.constr.x_min(8), param.ld2.constr.x_max(20); ...   u_th [0, 1]
    param.ld2.constr.x_min(9), param.ld2.constr.x_max(21) ...    u_wg [0, 1]
    ];

%% SOLVE FOR OPTIMAL SOLUTION

% Creating OCP
[ocp, info.L, info.E] = ocp__ld2_nox_EATS_whtc(x0, u0, xT, uT, t0, tf, param);

% Initial guess
v0 = initGuess__ld2_nox_EATS_whtc(x0, u0, t0, tf, param);

% Creating NLP and solving it
% Optimal solution is found when objective is within 'tol'.
nlpOpts.ipopt.constr_viol_tol   = 1e-4;
nlpOpts.ipopt.max_iter          = 20000;
nlpOpts.ipopt.tol               = 1e-6;

% If 'acceptable_iter' many iterations occur while not improving objective
% within 'acceptable_tol', an acceptable solutions is found.
nlpOpts.ipopt.acceptable_iter   = 100;
nlpOpts.ipopt.acceptable_tol    = 1e-5;

[ocp_sol, opt_v0]   = yopSolve_WHTC(ocp, v0, K, d, N, nlpOpts);

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
ld2_NOx_EATS = @(x, u, t) PT_PWL__ld2_nox_EATS_whtc(x, u, t, param, t0);
opt_signals = internalSignals__ld2_nox_EATS_whtc(ld2_NOx_EATS, t_opt, x_opt, u_opt);
opt_signals.info = info;

% Plotting
%ld2Plots(opt_signals, engine_map);
noxPlots(opt_signals, engine_map);
whtcPlots(v0, opt_signals, param)

% Printing comparable data
fprintf('\nWHTC from %.0f to %.0f seconds\n',t0,opt_signals.t_vec.val(end));
fprintf('Fuel consumption: %.2f ml/s\n', litrePerSecond(opt_signals).*1e3);
fprintf('Total fuel consumed: %.0f g\n', trapz(opt_signals.t_vec.val, opt_signals.W_f.val).*1e3);
fprintf('Total NOx: %.4f g\n', opt_signals.NOx_tot.val(end));
energy_consumed = trapz(opt_signals.t_vec.val, opt_signals.W_f.val).*param.ld2.q_HV;
fprintf('Energy consumed: %.2f MJ\n', energy_consumed.*1e-6)

E = zeros(length(opt_signals.t_vec.val),1);
for i = 2:length(opt_signals.t_vec.val)
    E(i) = trapz(opt_signals.t_vec.val(1:i), opt_signals.W_f.val(1:i)).*param.ld2.q_HV;
end
plot(opt_signals.t_vec.val, E)

%------------- END OF CODE --------------
% NAME: MAIN_ECMS__hev_whtc
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
%       createParam__hev_whtc
%       ocp__hev_whtc
%   	initGuess__hev_whtc
%       yopSolve_WHTC
%       PT_PWL__hev_whtc
%       internalSignals__hev_whtc
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
% Mars 2018; Last revision: 29-Mars-2018

%------------- BEGIN CODE --------------

close all
clc

%% VARIABLES

% Steady state idling in these points in WHTC:
% 0 -> 45 -> 140 -> 220 -> 350 -> 650 -> 720 -> 898 -> 1170 -> 1800

t0      = 0;            % Start time [s]
tf      = 45;            % End time   [s] 

N_init  = 500;          % Initial engine speed for steady state values
M_init  = 0;            % Initial engine torque for steady state values

K       = 1*(tf-t0);    % Collocation segments (normally 1 or 2 per second)
d       = 5;            % Collocation points (normally = 5)
N       = 1;            % Collocation intervals per segment (normally = 1)

%% INITIALIZE AND LOAD PARAMS

load('ld2_eng_map.mat', 'engine_map')

% Only create 'param' if it does not already exist to save time
if ~exist('param', 'var')
    regen_brake = 1000;
    param = createParam__hev_whtc(engine_map, regen_brake);
end

% Retrieve closest values for steady state operation for given torque and speed
[~, N_index_init] = min(abs( engine_map.rpm_range - N_init ));
[~, M_index_init] = min(abs( engine_map.torque_grid(:, N_index_init) - M_init ));

%% INITIAL STATES AND CONTROLS

% States
x0 = [engine_map.state_calibration{M_index_init, N_index_init}, ... ld2 x0 min
    engine_map.state_calibration{M_index_init, N_index_init} .* 1; ... ld2 x0 max
    param.ld2.constr.x_min(5), param.ld2.constr.x_min(5); ... NOx min, max [kg]
    N_init,	N_init; ...     Engine speed min, max [rpm]
    0.375, 0.375 ...        SOC min, max
    ];
% Controls
u0 = [engine_map.control_calibration{M_index_init, N_index_init}(1), ... u_f min
    engine_map.control_calibration{M_index_init, N_index_init}(1) .* 1; ... u_f max
    1, 1; ...               Throttle min, max
    0, 0; ...               Wastegate min, max
    1e-4, 1e-4 ...                M_em min, max
    ]; 

%% TERMINAL STATES AND CONTROLS

% states (( LEAVE FROM MIN TO MAX ))
xT = [param.ld2.constr.x_min(1), param.ld2.constr.x_max(1); ... p_cac [0.1, 5] bar
    param.ld2.constr.x_min(2), param.ld2.constr.x_max(2); ...   p_im [0.1, 5] bar
    param.ld2.constr.x_min(3), param.ld2.constr.x_max(3); ...   p_em [0.4, 5] bar
    param.ld2.constr.x_min(4), param.ld2.constr.x_max(4); ...   w_tc [~1, ~12] krpm
    param.ld2.constr.x_min(5), param.ld2.constr.x_max(5); ...   NOx [0, 1] kg
    param.ld2.constr.x_min(6), param.ld2.constr.x_max(6); ...   N [400, 2500] rpm
    0.25, 0.5 ...                                               SOC [0.25 0.5]
    ];
    
% controls (( LEAVE FROM MIN TO MAX ))
uT = [param.ld2.constr.x_min(8), param.ld2.constr.x_max(8); ... w_f [0, 280]
    param.ld2.constr.x_min(9), param.ld2.constr.x_max(9); ...   u_th [0, 1]
    param.ld2.constr.x_min(10), param.ld2.constr.x_max(10); ...    u_wg [0, 1]
    1e-4, 1e-4 ...    param.ld2.constr.x_min(11), param.ld2.constr.x_max(11) ...  M_em [-1500, 1500]
    ];

%% SOLVE FOR OPTIMAL SOLUTION

% Creating OCP
ocp = ocp__hev_whtc(x0, u0, xT, uT, t0, tf, param);
% Initial guess
v0 = ECMS__hev_whtc(x0, u0, t0, tf, param);

% Creating NLP and solving it
% Optimal solution is found when objective is within 'tol'.
nlpOpts.ipopt.constr_viol_tol   = 1e-4;
nlpOpts.ipopt.max_iter          = 4000;
nlpOpts.ipopt.tol               = 1e-6;
% If 'acceptable_iter' many iterations occur while not improving objective
% within 'acceptable_tol', an acceptable solutions is found.
nlpOpts.ipopt.acceptable_iter   = 80;
nlpOpts.ipopt.acceptable_tol    = 1e-5;

[ocp_sol, opt_v0] = yopSolve_WHTC(ocp, v0, K, d, N, nlpOpts);

%% PLOTTING
close all;

t_opt           = ocp_sol.getTimeVec(1) + t0;
x_opt           = ocp_sol.getStateMat(1);
u_opt           = ocp_sol.getControlMat(1);
u_opt(end+1, :) = u_opt(end, :);

% Simulating optimal solution to retrieve optimal signals
hev = @(x, u, t) PT_PWL__hev_whtc(x, u, t, param, t0);
opt_signals = internalSignals__hev_whtc(hev, t_opt, x_opt, u_opt);

% Plotting functions
%ld2Plots(opt_signals, engine_map);
%noxPlots(opt_signals, engine_map);
emPlots(v0, opt_signals);
%hevPlots(opt_signals);
whtcPlots(v0, opt_signals, param)

% Printing comparable data
fprintf('\nWHTC from %.0f to %.0f seconds\n',t0,opt_signals.t_vec.val(end));
fprintf('Fuel consumption: %.2f ml/s\n', litrePerSecond(opt_signals).*1e3);
fprintf('Total fuel consumed: %.0f g\n', trapz(opt_signals.t_vec.val, opt_signals.W_f.val).*1e3);
fprintf('SOC: %.1f to %.1f percent\n', ...
    opt_signals.SOC.val(1).*100, opt_signals.SOC.val(end).*100);
fprintf('Total NOx: %.4f g\n', opt_signals.NOx_tot.val(end));
energy_consumed = trapz(opt_signals.t_vec.val, opt_signals.W_f.val).*param.ld2.q_HV - ...
    (opt_signals.x_vec.val(end,7)-opt_signals.x_vec.val(1,7)).*param.batpack.Q.*param.batpack.Vnom;
fprintf('Energy consumed: %.2f MJ\n', energy_consumed.*1e-6)

E = zeros(length(opt_signals.t_vec.val),1);
for i = 2:length(opt_signals.t_vec.val)
    E(i) = trapz(opt_signals.t_vec.val(1:i), opt_signals.W_f.val(1:i)).*param.ld2.q_HV - ...
 	(opt_signals.x_vec.val(i,7)-opt_signals.x_vec.val(1,7)).* ...
    param.batpack.Q.*param.batpack.Vnom./param.batpack.series_cells;
end
plot(opt_signals.t_vec.val, E)

%------------- END OF CODE --------------
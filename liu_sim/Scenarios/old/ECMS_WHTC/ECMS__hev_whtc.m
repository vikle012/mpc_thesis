function v0 = ECMS__hev_whtc(x0, u0, t0, tf, param)
% FUNCTION: Simulating guessed control signals for initial guess of states
%           for the OCP.
%
% SYNTAX: output1 = initGuess__hev_whtc(input1, input2, input3,
%                                                   input4, input5);
%
% INPUTS:
%   input1 - Initial values of state vector
%   input2 - Initial values of control vector
%   input3 - Time of simulation start
%   input4 - Time of simulation ending
%   input5 - Struct containing all parameters and look-up tables
%
% OUTPUTS:
%   output1 - YOP-struct contaning all data for initial guess of
%             optimization problem
%
% EXAMPLE:
%   v0 = initGuess__hev_whtc(x0, u0, 45, 1170, param);
%
% OTHER FILES REQUIRED:
%   .m files:
%       PT_IG__hev_whtc
%       yopInitialGuess
%
%   .mat files:
%       none
%
% SUBFUNCTIONS:
%   none
%
% Author: Hampus Andersson and Fredrik Andersson
% Mars 2018; Last revision: 04-April-2018

%------------- BEGIN CODE --------------

fprintf('Simulating initial guess with ECMS...\n\n');

% For easy indexing
[p_cac, p_im, p_em, w_tc, NOx, N, SOC] = enum(7);
[u_f, u_thr, u_wg, M_em] = enum(4);

% Initial values for states and controls
u = [u0(u_f,1); ...     % Fuel
    1; ...              % Throttle
    0; ...              % Wastegate
    1e-9 ...            % M_em
    ];
x = x0(:,1);

% ECMS lambda value
lambda = 1.81;

% Simulate step from t0 to tf to obtain state vector
hev = @(t,x) PT_ECMS__hev_whtc(x, u, t, param, lambda);
opts = odeset('RelTol', 0.002, 'AbsTol', 0.005, 'MaxStep', 3);
[t_sim, x_sim] = ode15s(hev, [t0, tf], x, opts);

% Control signal u_sim(1) = u_f and thee derivative u_guess(1) = du_f
% is overrided later on. Assumably, u_th = 1 and u_wg = 0 and thee
% derivatives equal zero.
u_sim = repmat(u', length(t_sim), 1);
u_guess = repmat([0 0 0 0], length(t_sim), 1);

% Manipulate u_f and M_em to match simulated
for idx = 1:length(t_sim)
    [u_sim(idx,u_f), u_sim(idx,M_em)] = ECMS_minH__hev_whtc(lambda, x_sim(idx,:), ...
        t_sim(idx), param);
end

% Finite differentiate for derivatives of control signals manipulated
for i = 2:length(t_sim)
    u_guess(i,u_f) = (u_sim(i,u_f) - u_sim(i-1,u_f)) ./ (t_sim(i) - t_sim(i-1));
    u_guess(i,M_em) = (u_sim(i,M_em) - u_sim(i-1,M_em)) ./ (t_sim(i) - t_sim(i-1));
end

% Add to vector for PWL formulation
x_guess = [x_sim, u_sim];

% YOP formulation (duration, time_vector, state_guess, control_guess, ...)
v0 = yopInitialGuess(tf-t0, t_sim-t0, x_guess, u_guess, 0);

%------------- END OF CODE --------------
end


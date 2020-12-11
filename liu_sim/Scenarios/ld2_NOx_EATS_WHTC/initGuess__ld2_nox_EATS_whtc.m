function v0 = initGuess__ld2_nox_EATS_whtc(x0, u0, t0, tf, param)
% FUNCTION: Simulating guessed control signals for initial guess of states
%           for the OCP.
%
% SYNTAX: output1 = initGuess__ld2_nox_EATS_whtc(input1, input2, input3,
%                                                input4, input5);
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
%   v0 = initGuess__ld2_nox_EATS_whtc(x0, u0, 45, 1170, param);
%
% OTHER FILES REQUIRED:
%   .m files:
%       PT_IG__ld2_nox_EATS_whtc
%       yopInitialGuess
%
%   .mat files:
%       none
%
% SUBFUNCTIONS:
%   none
%
% Author: Hampus Andersson and Fredrik Andersson
% April 2018; Last revision: 17-April-2018

%------------- BEGIN CODE --------------

fprintf('Simulating initial guess...\n\n');

[u_f, u_thr, u_wg] = enum(3);

% Initial values for states and controls
u = [u0(u_f,1); ...     % FUEL
    1; ...              % THROTTLE
    0 ...               % WASTEGATE
    ];
x = x0(:,1);

% Simulate step from t0 to tf
ld2_NOx = @(t,x) PT_IG__ld2_nox_EATS_whtc(x, u, t, param);
opts = odeset('RelTol', 1e-5, 'AbsTol', 1e-6, 'MaxStep', 5);
[t_sim, x_sim] = ode15s(ld2_NOx, [t0, tf], x, opts);

% Control signal u_sim(1) = u_f and thee derivative u_guess(1) = du_f
% is overrided later on. Assumably, u_th = 1 and u_wg = 0 and thee
% derivatived equal zero.
u_sim = repmat(u', length(t_sim), 1);
u_guess = repmat([0 0 0], length(t_sim), 1);

% Polynomial to adjust steady state fuel flow according to engine speed for
% enhanced initial guess
ss_fuel = polyval([3.27386042788593e-06 ...
    -0.000266532098389988 ...
    7.42584402079555], x_sim(:,6));

% Override u_f with WHTC torque/10 + ss_fuel as initial guess.
% Adjust du_f accordingly.
lut_M = param.WHTC.lut_M;

u_sim(1,1) = max(0, full(lut_M(t_sim(1))) ./ 10 + ss_fuel(1));
for i = 2:length(t_sim)
    u_sim(i,1) = max(0, full(lut_M(t_sim(i))) ./ 10 + ss_fuel(i));
    u_guess(i,1) = (u_sim(i,1) - u_sim(i-1,1)) ./ (t_sim(i) - t_sim(i-1));
end

% Add to vector for PWL formulation
x_guess = [x_sim, u_sim];

% YOP formulation
v0 = yopInitialGuess(tf-t0, t_sim-t0, x_guess, u_guess, 0);
end


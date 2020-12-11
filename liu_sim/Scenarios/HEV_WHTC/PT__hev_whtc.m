function [dX, constraints, signals, param] = PT__hev_whtc(X, U, t, param, t0)
% FUNCTION: Wrapper for the model equations describing the dynamics of the
%           model.
%
% SYNTAX: [output1, output2, output3, output4] =
%           initGuess__hev_whtc(input1, input2, input3, input4, input5);
%
% INPUTS:
%   input1 - Current states
%   input2 - Current controls
%   input3 - Current time (used for fetching in look-up tables)
%   input4 - Struct containing all parameters and look-up tables
%   input5 - Start time of simulation/optimization
%
% OUTPUTS:
%   output1 - Derivative of states vector
%   output2 - Vector with constraints in the model (<0 = violation)
%   output3 - Struct with all intresting signals to view later on
%   output4 - Struct containing all parameters and look-up tables
%
% EXAMPLE:
%   hev = @(x,u,t) PT__hev_whtc(x, u, t, param, 0);
%   opts = odeset('RelTol', 1e-5, 'AbsTol', 1e-6, 'MaxStep', 5);
%   [t_sim, x_sim] = ode15s(hev, x, u, [t0, tf], opts);
%
% OTHER FILES REQUIRED:
%   .m files:
%       liu_diesel_NOx
%       electric_motor
%
%   .mat files:
%       none
%
% SUBFUNCTIONS:
%   none
%
% Author: Hampus Andersson and Fredrik Andersson
% Mars 2018; Last revision: 17-April-2018

%------------- BEGIN CODE --------------

% For easy indexing
[p_cac, p_im, p_em, w_tc, NOx, N, SOC] = enum(7);   % ICE and EM states
[u_f, u_thr, u_wg, M_em] = enum(4);         % ICE and EM controls

% Call of models for dynamics, constraints and signals
[dX_ld2, c_ld2, signals.ld2] = liu_diesel_NOx(X(p_cac:N), U(u_f:u_wg), param.ld2);    % ICE
[dX_em, c_em, signals.em] = electric_motor(X(N:SOC), U(M_em), param);                 % EM

% Add derivative of engine speed with P-controller
lut_dN = param.WHTC.lut_dN;
kp = 10;
lut_N = param.WHTC.lut_N;
dX_N = full(lut_dN(t+t0)) + kp .* (full(lut_N(t+t0)) - X(N));

% Add to vector in the right order
dX = [dX_ld2; ...
    signals.ld2.NOx_EO; ...
    dX_N; ...
    dX_em ...
    ];

% Add torque constraint to follow cycle
lut_M = param.WHTC.lut_M;
M_dem = full(lut_M(t+t0));

constraints = [c_ld2; ...
    c_em; ...
    M_dem - signals.ld2.M_ice - U(M_em); ...
    signals.ld2.M_ice + U(M_em) - M_dem ...
    ];

% Hybrid signals
signals.hev.M_ratio = signals.em.M_em ./ signals.ld2.M_ice;
signals.hev.M_em_norm = signals.em.M_em ./ (sqrt(signals.em.M_em.^2) + ...
    sqrt(signals.ld2.M_ice.^2));
signals.hev.M_ice_norm = signals.ld2.M_ice ./ (sqrt(signals.em.M_em.^2) + ...
    sqrt(signals.ld2.M_ice.^2));
signals.hev.M_tot = signals.em.M_em + signals.ld2.M_ice;

% Adding all signals to 'signals' from 'signals.em' and from 'signals.ld2'
f = fieldnames(signals.em);
for fieldCounter = 1:length(f)
    signals.(f{fieldCounter}) = signals.em.(f{fieldCounter});
end
f = fieldnames(signals.ld2);
for fieldCounter = 1:length(f)
    signals.(f{fieldCounter}) = signals.ld2.(f{fieldCounter});
end
f = fieldnames(signals.hev);
for fieldCounter = 1:length(f)
    signals.(f{fieldCounter}) = signals.hev.(f{fieldCounter});
end

%------------- END OF CODE --------------
end

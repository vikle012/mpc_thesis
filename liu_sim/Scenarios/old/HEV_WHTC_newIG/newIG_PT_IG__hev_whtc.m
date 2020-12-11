function [dX, constraints, signals, param] = newIG_PT_IG__hev_whtc(X, U, t, param, percent)
% FUNCTION: Description of purpose of function
%
% SYNTAX: [output1, output2, output2, output2] = newIG_PT_IG__hev_whtc(...
%                               input1, input2, input3, input4, input5);
%
% INPUTS:
%   input1 - State vector
%   input2 - Control vector
%   input3 - Current time
%   input4 - Struct with all parameters and look-up tables
%   input5 - Torque-split percentage
%
% OUTPUTS:
%   output1 - Derivative of state vector
%   output2 - Constraints vector
%   output3 - Internal signals vector
%   output4 - Struct with all parameters and look-up tables
%
% EXAMPLE:
%   [dX, c, s, param] = newIG_PT_IG__hev_whtc(X, U, t, param, 0.28);
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
% April 2018; Last revision: 13-April-2018

%------------- BEGIN CODE --------------

[p_cac, p_im, p_em, w_tc, NOx, N, SOC] = enum(7);   % ICE and EM states
[u_f, u_thr, u_wg, M_em] = enum(4);         % ICE and EM controls

%% Rough initial guess on torque-split
lut_M = param.WHTC.lut_M;
M_dem = full(lut_M(t));

% Polynomial to adjust steady state fuel flow according to engine speed
ss_fuel = polyval([3.27386042788593e-06 ...
    -0.000266532098389988 ...
    7.42584402079555], X(N));

% Initial guess of fuel injection based on torque request
factor = percent./0.125;
percenti = percent - max(min( factor *(param.batpack.SOC - X(SOC)),percent),percent-1);
if M_dem < 0
   U(M_em) = M_dem;
   U(u_f) = 0;
else
    U(M_em) = M_dem.*percenti;
    U(u_f) = max(0, (M_dem.*(1-percenti)) ./ 10 + ss_fuel);
end

%% Run model
[dX_ld2, c_ld2, signals.ld2] = ...
    liu_diesel_NOx(X(p_cac:N), U(u_f:u_wg), param.ld2);               % ICE
[dX_em, c_em, signals.em] = electric_motor(X(N:SOC), U(M_em), param); % EM

% Add derivative of engine speed with P-controller for eliminating drift
lut_dN = param.WHTC.lut_dN;
lut_N = param.WHTC.lut_N;
Kp = 10;
dX_N = full(lut_dN(t)) + Kp .* (full(lut_N(t)) - X(N));

% Add all derivatives to dX vector
dX = [dX_ld2; ...
    dX_N; ...
    dX_em ...
    ];

% Add all constraints to constraints vector
constraints = [c_ld2; ...
    c_em ...
    ];

% Hybrid signals
signals.hev.M_ratio = signals.em.M_em./signals.ld2.M_ice;
signals.hev.M_em_norm = signals.em.M_em./(sqrt(signals.em.M_em.^2)+sqrt(signals.ld2.M_ice.^2));
signals.hev.M_ice_norm = signals.ld2.M_ice./(sqrt(signals.em.M_em.^2)+sqrt(signals.ld2.M_ice.^2));
signals.hev.M_tot = signals.em.M_em + signals.ld2.M_ice;

% Adding all signals to signals from 'signals.em' and from 'signals.ld2'
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

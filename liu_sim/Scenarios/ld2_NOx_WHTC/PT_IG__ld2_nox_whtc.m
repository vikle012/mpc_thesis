function [dX, constraints, signals, param] = PT_IG__ld2_nox_whtc(X, U, t, param)

[p_cac, p_im, p_em, w_tc, NOx, N] = enum(6);   % ICE and EM states
[u_f, u_thr, u_wg] = enum(3);         % ICE and EM controls

%% Rough initial guess on fuel injection
lut_M = param.WHTC.lut_M;
M_dem = full(lut_M(t));

% Polynomial to adjust steady state fuel flow according to engine speed
ss_fuel = polyval([3.27386042788593e-06 ...
    -0.000266532098389988 ...
    7.42584402079555], X(N));

% Initial guess of fuel injection based on torque request
U(u_f) = max(0, M_dem ./ 10 + ss_fuel);

%% Run model
[dX_ld2, c_ld2, signals] = ...
    liu_diesel_NOx(X(p_cac:N), U(u_f:u_wg), param.ld2);    % ICE

% Add derivative of engine speed with P-controller for eliminating drift
lut_dN = param.WHTC.lut_dN;
lut_N = param.WHTC.lut_N;
Kp = 10;
dX_N = full(lut_dN(t)) + Kp .* (full(lut_N(t)) - X(N));

% PWL formulation
dX = [dX_ld2; ...
    signals.NOx_EO; ...
    dX_N ...
    ];

constraints = [c_ld2 ...
    %M_dem - signals.M_ice ...
    %signals.em.M_em + signals.ld2.M_ice - M_dem ...
    ];

end

function [dX, constraints, signals, param] = PT__ld2_nox_whtc(X, U, t, param, t0)
% Function normally calls both ICE and EM, but EM is removed in this
% branch.

[p_cac, p_im, p_em, w_tc, NOx, N] = enum(9);   % ICE and EM states
[u_f, u_thr, u_wg] = enum(3);         % ICE and EM controls

[dX_ld2, c_ld2, signals] = liu_diesel_NOx(X(p_cac:N), U(u_f:u_wg), param.ld2);    % ICE

% Add derivative of engine speed with P-controller
lut_dN = param.WHTC.lut_dN;
kp = 10;
lut_N = param.WHTC.lut_N;
dX_N = full(lut_dN(t+t0)) + kp .* (full(lut_N(t+t0)) - X(N));

dX = [dX_ld2; ...
    signals.NOx_EO; ...
    dX_N ...
    ];

% Add torque constraint to follow cycle
lut_M = param.WHTC.lut_M;
M_dem = full(lut_M(t+t0));

constraints = [c_ld2; ...
    M_dem - signals.M_ice ...
    %signals.M_ice - M_dem ...
    ];


end

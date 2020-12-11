function [dX, constraints, signals, param] = PT_IG__ld2_nox_EATS_whtc(X, U, t, param)

[p_cac, p_im, p_em, w_tc, NOx, N, T_DOC_1, T_DOC_2, T_DPF_1, T_DPF_2, ...
    T_DPF_3, T_SCR_1, T_SCR_2, T_SCR_3, T_SCR_4, T_SCR_5, T_silencer_1, ...
    T_silencer_2] = enum(18);   % ICE and EM states

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

constraints = [c_ld2; ...
    M_dem - signals.M_ice ...
    ];

% Aftertreatment
V = [signals.W_tw; ...
    1e5; ... Ambient pressure or P_em?
    signals.T_atw; ...
    300 ... ~ Ambient temperature
    ];
U = [];

[dX_EATS, signals.EATS, ~] = EATS_model(X(T_DOC_1:T_silencer_2), V, U, param.EATS, signals);

% PWL formulation
dX = [dX_ld2; ...
    signals.EATS.NOx_TP_Cu; ...
    dX_N; ...
    dX_EATS ...
    ];

end

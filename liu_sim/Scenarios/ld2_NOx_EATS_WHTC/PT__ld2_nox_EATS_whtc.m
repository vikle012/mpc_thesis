function [dX, constraints, signals, param] = PT__ld2_nox_EATS_whtc(X, U, t, param, t0)

% For easy indexing
[p_cac, p_im, p_em, w_tc, NOx, N, T_DOC_1, T_DOC_2, T_DPF_1, T_DPF_2, ...
    T_DPF_3, T_SCR_1, T_SCR_2, T_SCR_3, T_SCR_4, T_SCR_5, T_silencer_1, ...
    T_silencer_2] = enum(18); % ICE states

[u_f, u_thr, u_wg] = enum(3); % ICE controls

% Call model
[dX_ld2, c_ld2, signals.ld2] = liu_diesel_NOx(X(p_cac:N), U(u_f:u_wg), param.ld2);

% Add derivative of engine speed with P-controller
lut_dN = param.WHTC.lut_dN;
kp = 10;
lut_N = param.WHTC.lut_N;
dX_N = full(lut_dN(t+t0)) + kp .* (full(lut_N(t+t0)) - X(N));

% Add torque constraint to follow cycle
lut_M = param.WHTC.lut_M;
M_dem = full(lut_M(t+t0));

constraints = [c_ld2; ...
    M_dem - signals.ld2.M_ice; ...   % Lower limit
    signals.ld2.M_ice - M_dem ...      % Upper limit
    ];

%--------------------------------------------------------------------------
% Aftertreatment
%--------------------------------------------------------------------------
% Temperatures
V = [signals.ld2.W_tw; ...
    1e5; ... Ambient pressure or P_em? (( P_em = P_amb ))
    signals.ld2.T_atw; ...
    300 ... ~ Ambient temperature
    ];
U = [];

[dX_EATS, signals.EATS, ~] = EATS_model(X(T_DOC_1:T_silencer_2), V, ...
    U, param.EATS, signals.ld2);

dX = [dX_ld2; ...
    signals.EATS.NOx_TP_Cu; ...
    dX_N; ...
    dX_EATS ...
    ];

% Adding all signals to 'signals' from 'signals.EATS' and from 'signals.ld2'
f = fieldnames(signals.EATS);
for fieldCounter = 1:length(f)
    signals.(f{fieldCounter}) = signals.EATS.(f{fieldCounter});
end
f = fieldnames(signals.ld2);
for fieldCounter = 1:length(f)
    signals.(f{fieldCounter}) = signals.ld2.(f{fieldCounter});
end

end

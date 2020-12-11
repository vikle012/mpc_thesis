function [dX, constraints, signals, param] = PT_IG__hev_whtc(X, U, t, param)

[p_cac, p_im, p_em, w_tc, NOx, N, SOC] = enum(7);   % ICE and EM states
[u_f, u_thr, u_wg, M_em] = enum(4);         % ICE and EM controls

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
    signals.ld2.NOx_EO; ...
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

end

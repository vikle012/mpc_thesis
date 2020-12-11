function [dX, constraints, signals, param] = hybridPowertrain(X, U, t, param)


[p_cac, p_im, p_em, w_tc, NOx, N, SOC] = enum(7);   % ICE and EM states
[u_f, u_thr, u_wg, M_em] = enum(4);         % ICE and EM controls

[dX_ld2, c_ld2, signals.ld2] = liu_diesel_NOx(X(p_cac:N), U(u_f:u_wg), param.ld2);    % ICE
[dX_em, c_em, signals.em] = electric_motor(X(N:SOC), U(M_em), param);                 % EM

% Add derivative of engine speed
lut_dN = param.lut_dN_WHTC;
dX_N = full(lut_dN(t));

dX = [dX_ld2; ...
    dX_N; ...
    dX_em ...
    ];

lut_M = param.lut_M_WHTC;
M_dem = full(lut_M(t));

constraints = [c_ld2; ...
    c_em; ...
    M_dem - signals.em.M_em - signals.ld2.M_ice ...
    ];

% Hybrid signals
signals.hev.M_ratio = signals.em.M_em./signals.ld2.M_ice;
signals.hev.M_em_norm = signals.em.M_em./(sqrt(signals.em.M_em.^2)+sqrt(signals.ld2.M_ice.^2));
signals.hev.M_ice_norm = signals.ld2.M_ice./(sqrt(signals.em.M_em.^2)+sqrt(signals.ld2.M_ice.^2));
signals.hev.M_tot = signals.em.M_em + signals.ld2.M_ice;

% Adding all signals to z from 'em' and from 'ld2' is done by the loops
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

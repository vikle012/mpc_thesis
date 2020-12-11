function [dX, constraints, signals, param] = hybridPowertrain_pwl(X, U, t, param)
% The controls are states, and the control
% derivative is controlled instead, this way the control input is piecewise
% linear (pwl) when it is used for numerical optimal control.


[p_cac, p_im, p_em, w_t, NOx, N, SOC, u_f, u_thr, u_wg, M_em] = enum(11);
[Du_f, Du_thr, Du_wg, DM_em] = enum(4);

[dX_hev, constraints, signals, param] = hybridPowertrain(X(p_cac:SOC), X(u_f:M_em), t, param);

signals.Du_f    = U(Du_f);
signals.Du_thr  = U(Du_thr);
signals.Du_wg   = U(Du_wg);
signals.DM_em   = U(DM_em);

dX = [dX_hev; U];

end


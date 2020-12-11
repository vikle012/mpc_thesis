function [dX, constraints, signals, param] = test_LD2_pwl(X, U, N_ice, param)
% liu diesel 2 extension. The controls are states, and the control
% derivative is controlled instead, this way the control input is piecewise
% linear (pwl) when it is used for numerical optimal control.

[p_cac, p_im, p_em, w_t, T_at, u_f, u_thr, u_wg] = enum(8);
[Du_f, Du_thr, Du_wg] = enum(3);

[dX_ld2, constraints, signals, param] = liu_diesel_2(X(p_cac:T_at), X(u_f:u_wg), N_ice, param);

signals.Du_f = U(Du_f);
signals.Du_thr = U(Du_thr);
signals.Du_wg = U(Du_wg);

dX = [dX_ld2;0; U];

end

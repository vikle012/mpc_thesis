function [dX, constraints, signals, param] = PT_PWL__hev_eats_ss_free_N(X, U, t, param, t0)
% The controls are states, and the control
% derivative is controlled instead, this way the control input is piecewise
% linear (pwl) when it is used for numerical optimal control.

% States and controls
[p_cac, p_im, p_em, w_t, NOx, SOC, T_DOC_1, T_DOC_2, T_DPF_1, T_DPF_2, ...
    T_DPF_3, T_SCR_1, T_SCR_2, T_SCR_3, T_SCR_4, T_SCR_5, T_silencer_1, ...
    T_silencer_2, u_f, u_thr, u_wg, M_em, N] = enum(23);

% Derivative of controls which are controlled
[Du_f, Du_thr, Du_wg, DM_em, DN] = enum(5);

[dX_hev, constraints, signals, param] = ...
    PT__hev_eats_ss_free_N(X(p_cac:T_silencer_2), X(u_f:N), t, param, t0);

% Add derivatives to signals
signals.Du_f    = U(Du_f);
signals.Du_thr  = U(Du_thr);
signals.Du_wg   = U(Du_wg);
signals.DM_em   = U(DM_em);
signals.DN      = U(DN);

dX = [dX_hev; U];

end


 function [dX, constraints, signals, param] = PT_PWL__ld2_nox_EATS_tipin_whtc(X, U, t, param, t0)
% The controls are states, and the control
% derivative is controlled instead, this way the control input is piecewise
% linear (pwl) when it is used for numerical optimal control.

% States and controls
[p_cac, p_im, p_em, w_t, NOx, N, T_DOC_1, T_DOC_2, T_DPF_1, T_DPF_2, ...
    T_DPF_3, T_SCR_1, T_SCR_2, T_SCR_3, T_SCR_4, T_SCR_5, T_silencer_1, ...
    T_silencer_2, u_f, u_thr, u_wg] = enum(21);

% Derivative of controls which are controlled
[Du_f, Du_thr, Du_wg] = enum(3);

[dX_hev, constraints, signals, param] = ...
    PT__ld2_nox_EATS_tipin_whtc(X(p_cac:T_silencer_2), X(u_f:u_wg), t, param, t0);

% Add derivatives to signals
signals.Du_f    = U(Du_f);
signals.Du_thr  = U(Du_thr);
signals.Du_wg   = U(Du_wg);

dX = [dX_hev; U];
end


function [dXds, constraints, signals, param] = LD2_pwl_ds(X_and_v, U, N_ice, param)
% liu diesel 2 extension. The controls are states, and the control
% derivative is controlled instead, this way the control input is piecewise
% linear (pwl) when it is used for numerical optimal control. For
% LD2_pwl_ds, the independent variable is position, s, instead of time, t.
% v is the velocity.

% Not pretty, but makes other code clean
X = X_and_v(1:7);
v = X_and_v(8);

[dX, constraints, signals, param] = LD2_pwl(X, U, N_ice, param);
dXds = dX/v;

end

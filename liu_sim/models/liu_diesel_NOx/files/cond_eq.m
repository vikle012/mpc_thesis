function y = cond_eq(x, y_neg, y_pos, L, k, x0)
% A conditional equation that is differentiable. Takes y_neg if x<x0
% otherwise y_pos. The function is based on the logistic function which is
% controlled through the parameters L, k, x0. See wikipedia for an
% explanation of the logistic function.

fx = logistic_function(x, L, k, x0);
y = y_neg + fx .* (y_pos - y_neg);

end

function fx = logistic_function(x, L, k, x0)
% x0 = the x-value of the sigmoid's midpoint,
% L  = the curve's maximum value, and
% k  = the steepness of the curve.
e = exp(1);
fx = L ./ ( 1 + e.^( -k*(x-x0) ) );
end
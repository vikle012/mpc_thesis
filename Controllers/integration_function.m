function F = integration_function(f, method, Ts, M)
% INTEGRATION_FUNCTION: 
% Formulates discrete time dynamics using different numerical integrations
%
% Valid options: 
% - Forward Euler method ("EF")
% - Runge-Kutta 4th order ("RK4")

addpath('C:\Users\Jonte\Documents\GitHub\mpc_thesis\CasADi\casadi-windows-matlabR2016a-v3.5.5')
import casadi.*

h = Ts/M; % Step size
X0 = casadi.MX.sym('X0', 5);
U = casadi.MX.sym('U', 3);
X = X0;
Q = 0;   % Objective cost function

if method == "EF"
    for k = 1:M
        [k1, k1_q] = f(X, U);
        X = X + h*k1;
        Q = Q + h*k1_q;
    end
elseif method == "RK4"
    for k = 1:M 
        [k1, k1_q] = f(X, U);
        [k2, k2_q] = f(X + h/2*k1, U);
        [k3, k3_q] = f(X + h/2*k2, U);
        [k4, k4_q] = f(X + h*k3, U);
        X = X + h/6*(k1 + 2*k2 + 2*k3 + k4);
        Q = Q + h/6*(k1_q + 2*k2_q + 2*k3_q + k4_q);
    end
else
    error('Invalid integration method.');
end

F = casadi.Function('F', {X0, U}, {X, Q}, {'x0','p'}, {'xf', 'qf'});

end


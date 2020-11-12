% Example 8.13 in Model Predictive Control: Theory ...
% Page 582

clear all
close all
clc

addpath('C:\Users\Jonte\Documents\GitHub\mpc_thesis\CasADi\casadi-windows-matlabR2016a-v3.5.5')
import casadi.*

f_c = @(x,u)[x(2); -x(1) - x(1)^3 + u];

% Decision variable
N = 40;
U = SX.sym('U', N);
Q1 = diag([10 5]);

% System simulation
xk = [1; 0];
c = 0;
for k=1:N
    % Runge-Kutta fourth order
    dt = 0.2;
    k1 = f_c(xk, U(k));
    k2 = f_c(xk+0.5*dt*k1, U(k));
    k3 = f_c(xk+0.5*dt*k2, U(k));
    k4 = f_c(xk+dt*k3, U(k));
    xk = xk + dt/6.0*(k1 + 2*k2 + 2*k3 + k4);
    % Add contribution to objective function
    c = c + 10*xk(1)^2 + 5*xk(2)^2 + U(k)^2;
end
% Terminal constraint
G = xk - [0; 0]; 

% Create an NLP solver object
nlp = struct('x', U, 'f', c, 'g', G);
solver = nlpsol('solver', 'ipopt', nlp);
% Solve the NLP
solution = solver('x0', 0, 'lbx', -1, 'ubx', 1, 'lbg', 0, 'ubg', 0);
U_opt = full(solution.x);

% Plot (egen kod)
tgrid = linspace(0, dt, N);
plot(tgrid, U_opt, '--')
xlabel('t')
legend('u')

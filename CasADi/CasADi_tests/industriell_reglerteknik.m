clear all
close all
clc

addpath('C:\Users\Jonte\Documents\GitHub\mpc_thesis\CasADi\casadi-windows-matlabR2016a-v3.5.5')
import casadi.*

% Cont. time dynamics
f_c = @(x,u)[-0.0556*x(1) - 0.05877*x(2); 0.01503*x(1) - 0.005887*x(2) - 0.03384*u];

% Decision variable
N = 20;
U = SX.sym('U', N);

%Q1 = eye(2);
%Q2 = 0.1;

Ts = 1;
x0 = [20; 2];   % init

% System simulation
xk = x0;        % changing xk
c = 0;          % objective function

for k=1:N
    % Runge-Kutta fourth order
    dt = Ts; 
    k1 = f_c(xk, U(k));
    k2 = f_c(xk+0.5*dt*k1, U(k));
    k3 = f_c(xk+0.5*dt*k2, U(k));
    k4 = f_c(xk+dt*k3, U(k));
    xk = xk + dt/6.0*(k1 + 2*k2 + 2*k3 + k4);
    
    % Add contribution to objective function
    c = c + 1*xk(1)^2 + 1*xk(2)^2 + 0.1*U(k)^2;
end
%%%%%%%%%%%%%%%%%%%%
% Terminal constraint
G = xk - [0; 0]; 

qp = struct('x', U, 'f', c, 'g', G);
solver = casadi.qpsol('solver', 'qpoases', qp);
% Solve the QP problem

solution = solver('x0', 1,'lbx', -1, 'ubx', 1); %, 'lbg', 0, 'ubg', 0);
U_opt = full(solution.x);

tgrid = linspace(0, dt, N);
stairs(tgrid, U_opt)

[t_sim, x_sim] = ode15s(@(t,x)f_opt(t, x, dt, N, U_opt, f_c), ...
                [tgrid(1) 60], x0);
            
figure(2)
plot(t_sim, x_sim)

function dx = f_opt(t, x, dt, N, u_opt, f_c)

u_opt = [u_opt(1); u_opt];
tgrid = linspace(0, dt, N+1);
u = interp1(tgrid,u_opt,t,'next','extrap');
if t >= tgrid(end)
    u = u_opt(end);
end
dx = f_c(x, u);

end
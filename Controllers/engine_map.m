% OCP calculates x_opt and u_opt during stationarity
% while providing sufficient torque  
clear
close all
clc

% Change path if necessary
addpath('C:\Users\Jonte\Documents\GitHub\mpc_thesis\MATLAB_model')
addpath('C:\Users\Jonte\Documents\GitHub\mpc_thesis\Controllers')
addpath('C:\Users\Jonte\Documents\GitHub\mpc_thesis\WHTC')
addpath('C:\Users\Jonte\Documents\GitHub\mpc_thesis\WahlstromErikssonTCDI_EGR_VGT')
addpath('C:\Users\Jonte\Documents\GitHub\mpc_thesis\CasADi\casadi-windows-matlabR2016a-v3.5.5')
import casadi.*
load parameterData

% Declare states
x = casadi.SX.sym('X',5); % 5x1 matrix
% p_im    = x(1);
% p_em    = x(2);
% X_Oim   = x(3);
% X_Oem   = x(4);
% w_t     = x(5);

% Declare control signals
u = casadi.SX.sym('U',3); % 3x1 matrix
% u_delta = u(1);
% u_egr   = u(2);
% u_vgt   = u(3);

% Declare external input signals
n_e = 1500;     % Note: 500 <= n_e <= 2000

% Calculate M_e and other saturated signals using CasADi variables

[x_dot, y, constr] = diesel_engine(x, u, n_e, model);

M_e = y.M_e;            % M_e >= m
m = 800;

% Weights (nom with average from min, max values)
p_amb = model.p_amb;
q1 = 1/((0.5*p_amb + 10*p_amb)/2)^2;
q2 = 1/((0.5*p_amb + 20*p_amb)/2)^2;
q3 = 1/((0 + 1)/2)^2;
q4 = 1/((0 + 1)/2)^2;
q5 = 1/((100*pi/30 + 200000*pi/30)/2)^2;

r1 = 100/((1 + 250)/2)^2;
r2 = 1/((0 + 100)/2)^2;
r3 = 1/((20 + 100)/2)^2;

Q = diag([q1 q2 q3 q4 q5]);
R = diag([r1 r2 r3]);

% Formulate the NLP
f = x'*Q*x + u'*R*u;                        % Objective/cost function
g = [M_e; x_dot; constr.constraints];       % Constraint variables
w = [x; u];                                 % Optimization variables

% ipopt options
% opts.ipopt.max_iter = 500;
% opts.ipopt.print_level = 0;
opts.ipopt.acceptable_tol = 1e-8;       % 1e-6 by default
opts.ipopt.constr_viol_tol = 1e-8;      % 1e-6 by default
% opts.ipopt.bound_relax_factor = 0;    % 1e-8 by default

% Create solver
nlp = struct('x', w, 'f', f, 'g', g);
solver = casadi.nlpsol('solver', 'ipopt', nlp, opts);

% Initial values/guess
w0  = [1.0e+05*1.9307; 1.0e+05*2.1069; 0.2120; 0.1109; 7.5804e+03; 127.4860; 63.2446; 40.5904];                  
% Constraints on opt. variables
lbw = [0.5*p_amb; 0.5*p_amb; 0; 0; 100*pi/30; 1; 0; 20]; 
ubw = [10*p_amb; 20*p_amb; 1; 1; 200000*pi/30; 250; 100; 100];
% Constraints
lbg = [m; zeros(5,1); constr.lbg];            
ubg = [inf; zeros(5,1); constr.ubg];

% Solve
solution = solver('x0',  w0, ...      % Initial guess
                  'lbx', lbw, ...     % Lower variable bound
                  'ubx', ubw, ...     % Upper variable bound
                  'lbg', lbg, ...     % Lower constraint bound
                  'ubg', ubg);        % Upper constraint bound

w_opt = full(solution.x);
x_opt = w_opt(1:5);
u_opt = w_opt(6:8);

% Test and aquire setpoints for later use
% [dx, sig, ~] = diesel_engine(x_opt, u_opt, n_e, model);
% x_egr_sp = sig.x_egr;
% lambda_O_sp = sig.lambda_O;
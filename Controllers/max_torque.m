% Calculates max torque for n_e = 1200 rpm 
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
n_e = 1200;     % Note: 500 <= n_e <= 2000

% Calculate M_e and other saturated signals using CasADi variables

[~, y, constr] = diesel_engine(x, u, n_e, model);
M_e = y.M_e;

% Formulate the NLP
f = -M_e;                        % Objective/cost function
g = [constr.constraints];       % Constraint variables
w = [x; u];                                 % Optimization variables

% Create solver
nlp = struct('x', w, 'f', f, 'g', g);
solver = casadi.nlpsol('solver', 'ipopt', nlp);

% Initial values/guess: running engine_map and increasing m for 1200 rpm
w0  = [2.7522e+05; 2.7522e+05; 0.2314; 0.0926; 9.6932e+03; 249.9777; 0.0013; 47.2118];  
% Constraints on opt. variables
lbw = [0.5*model.p_amb; 0.5*model.p_amb; 0; 0; 100*pi/30; 1; 0; 20]; 
ubw = [10*model.p_amb; 20*model.p_amb; 1; 1; 200000*pi/30; 250; 100; 100];
% Constraints
lbg = [constr.lbg];            
ubg = [constr.ubg];

% Solve
solution = solver('x0',  w0, ...      % Initial guess
                  'lbx', lbw, ...     % Lower variable bound
                  'ubx', ubw, ...     % Upper variable bound
                  'lbg', lbg, ...     % Lower constraint bound
                  'ubg', ubg);        % Upper constraint bound

w_opt = full(solution.x);
x_opt = w_opt(1:5);
u_opt = w_opt(6:8);
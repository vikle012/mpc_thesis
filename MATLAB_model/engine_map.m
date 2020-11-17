% Solves the following optimal control problem (OCP)
% 
% minimize     x'*Q*x + u'*R*u
%      x,u
% 
% subject to     M_e >= m
%
%             [ 1  ]    [u_delta]    [ 250 ]
%             | 0  | <= |u_egr  | <= | 100 |
%             [ 20 ]    [u_vgt  ]    [ 100 ]

% Questions:
% - Cost function including states
% - Contraints on n_e and x
% - Terminal constraint/time
% - Initial value

clear
close all
clc

% Change path if necessary
addpath('C:\Users\Jonte\Documents\GitHub\mpc_thesis\MATLAB_model')
addpath('C:\Users\Jonte\Documents\GitHub\mpc_thesis\WahlstromErikssonTCDI_EGR_VGT')
addpath('C:\Users\Jonte\Documents\GitHub\mpc_thesis\CasADi\casadi-windows-matlabR2016a-v3.5.5')
import casadi.*
load parameterData

%% Set-up

% Declare states
x = casadi.SX.sym('X',5);
% p_im    = x(1);
% p_em    = x(2);
% X_Oim   = x(3);
% X_Oem   = x(4);
% w_t     = x(5);

% Declare control signals
u = casadi.SX.sym('U',3);
% u_delta = u(1);
% u_egr   = u(2);
% u_vgt   = u(3);

% Declare external input signals
n_e = 1500;     % 500 <= n_e <= 2000

%% Calculate M_e using CasADi variables

[dx, y, ~] = diesel_engine(x, u, n_e, model);
M_e = y.M_e;

%% CasADi set-up

% Weights <---- vikta r?tt, men st?rre vikt p? u_delta
Q =
R =

% Formulate the NLP
f = Q*x^2 + R*u^2;         % Cost function
g = [M_e; dx];             % Constraints LHS

w = [x; u];

% Create solver
nlp = struct('x', w, 'f', f, 'g', g);
solver = casadi.nlpsol('solver', 'ipopt', nlp);

m = 750; % TEMP
args.x0  = [];                  % Initial input
args.lbx = [1; 0; 20];          % Constraints on input signals
args.ubx = [250; 100; 100];
args.lbg = [m; zeros(5,1)];            % Constraints
args.ubg = [inf; zeros(5,1)];

% Solve
solution = solver('x0', args.x0, ...       % Initial guess
                  'lbx', args.lbx, ...     % Lower variable bound
                  'ubx', args.ubx, ...     % Upper variable bound
                  'lbg', args.lbg, ...     % Lower constraint bound
                  'ubg', args.ubg);        % Upper constraint bound

u_opt = full(solution.x);
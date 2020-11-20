%% OCP calculates x_opt and u_opt during stationarity
% while providing sufficient torque  

clear
close all
clc

% Change path if necessary
addpath('C:\Users\Jonte\Documents\GitHub\mpc_thesis\MATLAB_model')
addpath('C:\Users\Jonte\Documents\GitHub\mpc_thesis\WahlstromErikssonTCDI_EGR_VGT')
addpath('C:\Users\Jonte\Documents\GitHub\mpc_thesis\CasADi\casadi-windows-matlabR2016a-v3.5.5')
import casadi.*
load parameterData

%% CasADi variables set-up

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

%% Calculate M_e and other saturated signals using CasADi variables

[x_dot, y, ~] = diesel_engine(x, u, n_e, model);

p_amb = model.p_amb;
BSR_opt = model.BSR_opt;
PI_egropt = model.PI_egropt;

M_e = y.M_e;            % M_e >= m

% Constraints from Simulink model
% From turbine.m
PI_t = y.PI_t;          % PI_t <= 0.9999            "Saturation"
BSR = y.BSR;            % 0 <= BSR <= 2*BSR_opt      Saturation
eta_tm = y.eta_tm;      % 0.2 <= eta_tm <= 1         Saturation

constr_t = [PI_t; BSR; eta_tm];
constr_t_lbg = [0; 0; 0.2];
constr_t_ubg = [0.9999; 2*BSR_opt; 1];

% From EGR_system.m
PI_egr = y.PI_egr;      % PI_egropt <= PI_egr <= 1   Saturation

constr_EGR = PI_egr;
constr_EGR_lbg = PI_egropt;
constr_EGR_ubg = 1;

% From cylinder.m
lambda_air = y.lambda_air;  % 1.2 < lambda_air      Discussed value
X_Oe = y.X_Oe;              % 0 <= X_Oe             "Saturation"

constr_cyl = [lambda_air; X_Oe];
constr_cyl_lbg = [1.2; 0];
constr_cyl_ubg = [inf; inf;];

% From compressor.m
PI_c = y.PI_c;          % 1 <= PI_c                 "Saturation"
W_c = y.W_c;            % 1e-4 <= W_c <= inf         Saturation
eta_c = y.eta_c;        % 0.2 <= eta_c              "Saturation"

constr_c = [PI_c; W_c; eta_c];
constr_c_lbg = [1; 1e-4; 0.2];
constr_c_ubg = [inf; inf; 1];

constr = [constr_t; constr_EGR; constr_cyl; constr_c];
constr_lbg = [constr_t_lbg; constr_EGR_lbg; constr_cyl_lbg; constr_c_lbg];
constr_ubg = [constr_t_ubg; constr_EGR_ubg; constr_cyl_ubg; constr_c_ubg];

%% CasADi NLP set-up

% Weights (nom with average from min, max values)
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
f = x'*Q*x + u'*R*u;        % Objective/cost function
g = [M_e; x_dot; constr];      % Constraint variables
w = [x; u];                 % Optimization variables

% ipopt options
opts.ipopt.max_iter = 500;
% opts.ipopt.print_level = 0;
opts.ipopt.acceptable_tol = 1e-8;   % 1e-6 by default
opts.ipopt.constr_viol_tol = 1e-8;  % 1e-6 by default
% opts.ipopt.bound_relax_factor = 0;  % 1e-8 by default

% Create solver
nlp = struct('x', w, 'f', f, 'g', g);
solver = casadi.nlpsol('solver', 'ipopt', nlp, opts);

m = 800; % TEMP

% Initial values/guess
args.w0  = [1.0e+05*1.9307; 1.0e+05*2.1069; 0.2120; 0.1109; 7.5804e+03; 127.4860; 63.2446; 40.5904];                  
% Constraints on opt. variables
args.lbw = [0.5*p_amb; 0.5*p_amb; 0; 0; 100*pi/30; 1; 0; 20]; 
args.ubw = [10*p_amb; 20*p_amb; 1; 1; 200000*pi/30; 250; 100; 100];
% Constraints
args.lbg = [m; zeros(5,1); constr_lbg];            
args.ubg = [inf; zeros(5,1); constr_ubg];

% Solve
solution = solver('x0',  args.w0, ...      % Initial guess
                  'lbx', args.lbw, ...     % Lower variable bound
                  'ubx', args.ubw, ...     % Upper variable bound
                  'lbg', args.lbg, ...     % Lower constraint bound
                  'ubg', args.ubg);        % Upper constraint bound

w_opt = full(solution.x);
x_opt = w_opt(1:5);
u_opt = w_opt(6:8);

% Testing
% [dx, sig, ~] = diesel_engine(x_opt, u_opt, n_e, model);

%% Linearizing system around x_opt and u_opt as stationary points
%  follows the format in "Reglerteori"

A = jacobian(diesel_engine(x, u, n_e, model), x);
B = jacobian(diesel_engine(x, u, n_e, model), u);

z = (x - x_opt);
v = (u - u_opt);
z_dot = A*z + B*v; % Might be f(x0, u0) =\ 0

%% Linearized MPC




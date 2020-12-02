clear
clc

% Change path if necessary
addpath('C:\Users\Jonte\Documents\GitHub\mpc_thesis\MATLAB_model')
addpath('C:\Users\Jonte\Documents\GitHub\mpc_thesis\Controllers')
addpath('C:\Users\Jonte\Documents\GitHub\mpc_thesis\WahlstromErikssonTCDI_EGR_VGT')
addpath('C:\Users\Jonte\Documents\GitHub\mpc_thesis\CasADi\casadi-windows-matlabR2016a-v3.5.5')
import casadi.*
load parameterData

%%

% From engine_map.m with m = 800
x_opt = [156530.169403718; 162544.519591941; 0.228497904007297; ...
         0.120767676163110; 6598.24892268606];
u_opt = [110.976447879888; 15.3657549771663; 70.8311084176616];    

% From engine_map.m with m = 1200
x_step = [190976.306096801; 190976.304195917; 0.231400000000089; ...
          0.107150994106105; 7865.28279123926];
u_step = [155.324792091864; 0.00144061709657587; 79.1879983060218];

% Declare external input signals
n_e = 1500;     % Note: 500 <= n_e <= 2000

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

[x_dot, y, ~] = diesel_engine(x, u, n_e, model);

% Linearizing system around x_opt and u_opt as stationary points
% follows the format in "Reglerteori"
A = casadi.Function('A', {x,u}, {jacobian(diesel_engine(x, u, n_e, model), x)});
B = casadi.Function('B', {x,u}, {jacobian(diesel_engine(x, u, n_e, model), u)});
xtilde = casadi.SX.sym('xtilde', 5);
utilde = casadi.SX.sym('xtilde', 3);

% Linearized continous time system
xtilde_dot = A(x_opt, u_opt)*xtilde + B(x_opt, u_opt)*utilde;

T = 5;     % Time horizon
N = 200;    % Prediction horizon/number of control intervals
Ts = T/N;   % Sample time

q1 = 10/((0.5*model.p_amb + 10*model.p_amb)/2)^2;
q2 = 10/((0.5*model.p_amb + 20*model.p_amb)/2)^2;
q3 = 10/((0 + 1)/2)^2;
q4 = 10/((0 + 1)/2)^2;
q5 = 10/((100*pi/30 + 200000*pi/30)/2)^2;

r1 = 1/((1 + 250)/2)^2;
r2 = 0.1/((0 + 100)/2)^2;
r3 = 0.1/((20 + 100)/2)^2;

Q1 = diag([q1 q2 q3 q4 q5]);
Q2 = diag([r1 r2 r3]);

% Objective function
L = xtilde'*Q1*xtilde + utilde'*Q2*utilde;

% Continuous time dynamics using diesel_engine.m
f = casadi.Function('f', {xtilde, utilde}, {xtilde_dot, L});

% Start with an empty NLP
w   = [];
w0  = [];
lbw = [];
ubw = [];
J   = 0;
G   = [];
lbg = [];
ubg = [];

x_lbw = [0.5*model.p_amb; 0.5*model.p_amb;  0; 0; 100*pi/30]; 
x_ubw = [10*model.p_amb;  20*model.p_amb;   1; 1; 200000*pi/30];
u_lbw = [1; 0; 20];
u_ubw = [250; 100; 100];
xtilde_lbw = x_lbw - x_opt; 
xtilde_ubw = x_ubw - x_opt;
utilde_lbw = u_lbw - u_opt;
utilde_ubw = u_ubw - u_opt;

xtilde_dagger = [-150000; 0; 0; 0; 0]; % Disturbed state
% xtilde_dagger = x_step-x_opt;

% "Lift" initial conditions
Xk = casadi.MX.sym('X0', 5);
w = [w; Xk];
lbw = [lbw; xtilde_dagger];
ubw = [ubw; xtilde_dagger];
w0 = [w0; xtilde_dagger];

F = integration_function(f, "EF", Ts, N);

% Formulate the NLP
for k=0:N-1        
    
    % New NLP variable for the control     
    Uk = casadi.MX.sym(['U_' num2str(k)], 3);
    w = [w; Uk];
    lbw = [lbw; utilde_lbw];
    ubw = [ubw; utilde_ubw]; 
    w0 = [w0; zeros(3,1)];

    % Integrate till the end of the interval
    Fk = F('x0', Xk, 'p', Uk);
    Xk_next = Fk.xf;
    J = J + Fk.qf;

    % New NLP variable for state at end of interval
    Xk = casadi.MX.sym(['X_' num2str(k+1)], 5);
    w = [w; Xk];
    lbw = [lbw; xtilde_lbw];
    ubw = [ubw; xtilde_ubw];
    w0 = [w0; zeros(5,1)];

    % Add equality constraint
    G = [G; Xk_next-Xk];
    lbg = [lbg; zeros(5,1)];
    ubg = [ubg; zeros(5,1)];  
    
end

% ipopt options
% opts.ipopt.max_iter = 1000;
opts.ipopt.acceptable_tol = 1e-8;   % 1e-6 by default
opts.ipopt.constr_viol_tol = 1e-8;  % 1e-6 by default

% Create an NLP solver function
nlp = struct('f', J, 'x', w, 'g', G);
solver = casadi.nlpsol('solver', 'ipopt', nlp, opts);

solution = solver('x0',  w0, ...      % Initial guess
                  'lbx', lbw, ...     % Lower variable bound
                  'ubx', ubw, ...     % Upper variable bound
                  'lbg', lbg, ...     % Lower constraint bound
                  'ubg', ubg);        % Upper constraint bound
              
w_opt = full(solution.x);

%% Extracting using xtilde_opt = x - x_opt

tgrid = linspace(0, T, N+1);
x1_opt = w_opt(1:8:end) + repmat(x_opt(1),N+1,1);
x2_opt = w_opt(2:8:end) + repmat(x_opt(2),N+1,1);
x3_opt = w_opt(3:8:end) + repmat(x_opt(3),N+1,1);
x4_opt = w_opt(4:8:end) + repmat(x_opt(4),N+1,1);
x5_opt = w_opt(5:8:end) + repmat(x_opt(5),N+1,1);
u1_opt = w_opt(6:8:end) + repmat(u_opt(1),N,1);
u2_opt = w_opt(7:8:end) + repmat(u_opt(2),N,1);
u3_opt = w_opt(8:8:end) + repmat(u_opt(3),N,1);

%% Plotting

figure(1)
subplot(511)
plot(tgrid, linspace(x_opt(1), x_opt(1), N+1), 'k--', 'Linewidth', 1.3)
hold on
plot(tgrid, x1_opt)
% plot(tgrid, linspace(x_lbw(1), x_lbw(1), N+1), 'r--')
% plot(tgrid, linspace(x_ubw(1), x_ubw(1), N+1), 'r--')
title('p_{im}')

subplot(512)
plot(tgrid, linspace(x_opt(2), x_opt(2), N+1), 'k--', 'Linewidth', 1.3)
hold on
plot(tgrid, x2_opt)
% plot(tgrid, linspace(x_lbw(2), x_lbw(2), N+1), 'r--')
% plot(tgrid, linspace(x_ubw(2), x_ubw(2), N+1), 'r--')
title('p_{em}')

subplot(513)
plot(tgrid, linspace(x_opt(3), x_opt(3), N+1), 'k--', 'Linewidth', 1.3)
hold on
plot(tgrid, x3_opt)
% plot(tgrid, linspace(x_lbw(3), x_lbw(3), N+1), 'r--')
% plot(tgrid, linspace(x_ubw(3), x_ubw(3), N+1), 'r--')
title('X_{Oim}')

subplot(514)
plot(tgrid, linspace(x_opt(4), x_opt(4), N+1), 'k--', 'Linewidth', 1.3)
hold on
plot(tgrid, x4_opt)
% plot(tgrid, linspace(x_lbw(4), x_lbw(4), N+1), 'r--')
% plot(tgrid, linspace(x_ubw(4), x_ubw(4), N+1), 'r--')
title('X_{Oem}')

subplot(515)
plot(tgrid, linspace(x_opt(5), x_opt(5), N+1), 'k--', 'Linewidth', 1.3)
hold on
plot(tgrid, x5_opt)
% plot(tgrid, linspace(x_lbw(5), x_lbw(5), N+1), 'r--')
% plot(tgrid, linspace(x_ubw(5), x_ubw(5), N+1), 'r--')
title('\omega_t')

figure(2)
subplot(311)
plot(tgrid, linspace(u_opt(1), u_opt(1), N+1), 'k--', 'Linewidth', 1.3)
hold on
stairs(tgrid, [u1_opt; nan])
% plot(tgrid, linspace(u_lbw(1), u_lbw(1), N+1), 'r--')
% plot(tgrid, linspace(u_ubw(1), u_ubw(1), N+1), 'r--')
title('u_\delta')

subplot(312)
plot(tgrid, linspace(u_opt(2), u_opt(2), N+1), 'k--', 'Linewidth', 1.3)
hold on
stairs(tgrid, [u2_opt; nan])
% plot(tgrid, linspace(u_lbw(2), u_lbw(2), N+1), 'r--')
% plot(tgrid, linspace(u_ubw(2), u_ubw(2), N+1), 'r--')
title('u_{egr}')

subplot(313)
plot(tgrid, linspace(u_opt(3), u_opt(3), N+1), 'k--', 'Linewidth', 1.3)
hold on
stairs(tgrid, [u3_opt; nan])
% plot(tgrid, linspace(u_lbw(3), u_lbw(3), N+1), 'r--')
% plot(tgrid, linspace(u_ubw(3), u_ubw(3), N+1), 'r--')
title('u_{vgt}')

% M_e = [];
% for i = 0:N-1
%     opt_x = [x1_opt(i+1) x2_opt(i+1) x3_opt(i+1) x4_opt(i+1) x5_opt(i+1)];
%     opt_u = [u1_opt(i+1) u2_opt(i+1) u3_opt(i+1)];
%     [~, sig, ~] = diesel_engine(opt_x, opt_u, n_e, model);
%     M_e = [M_e; sig.M_e];
% end
% 
% figure(3)
% plot(tgrid, [M_e; nan])
% hold on
% stairs(tgrid, [800*ones(T/2/Ts, 1); 1200*ones(T/2/Ts, 1); nan], 'k--')
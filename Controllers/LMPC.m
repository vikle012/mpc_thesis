clc
clear
load parameterData

% From engine_map.m with m = 800
x_opt = [156530.169403718; 162544.519591941; 0.228497904007297; ...
         0.120767676163110; 6598.24892268606];
u_opt = [110.976447879888; 15.3657549771663; 70.8311084176616];    

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

T = 5;  % Time horizon
N = 500; % Prediction horizon/number of control intervals

q1 = 1/((0.5*model.p_amb + 10*model.p_amb)/2)^2;
q2 = 1/((0.5*model.p_amb + 20*model.p_amb)/2)^2;
q3 = 1/((0 + 1)/2)^2;
q4 = 1/((0 + 1)/2)^2;
q5 = 10/((100*pi/30 + 200000*pi/30)/2)^2;

r1 = 10/((1 + 250)/2)^2;
r2 = 0.1/((0 + 100)/2)^2;
r3 = 0.1/((20 + 100)/2)^2;

Q1 = diag([q1 q2 q3 q4 q5]);
Q2 = diag([r1 r2 r3]);

% Objective function
L = xtilde'*Q1*xtilde + utilde'*Q2*utilde;
%L = q1*(y.x_egr - x_egr_sp)^2 + q2*(x(2) - x(1)) + q3*norm(u, 2)^2;

% Continuous time dynamics using diesel_engine.m
f = casadi.Function('f', {xtilde, utilde}, {xtilde_dot, L});

M = 4; % RK4 steps per interval
Ts = T/N; % Sample time
h = Ts/M; % Step size
X0 = casadi.MX.sym('X0', 5);
U = casadi.MX.sym('U', 3);
X = X0;
Q = 0;   % Objective cost function
% Formulate discrete time dynamics using RK4
for k=1:M 
    [k1, k1_q] = f(X, U);
    [k2, k2_q] = f(X + h/2*k1, U);
    [k3, k3_q] = f(X + h/2*k2, U);
    [k4, k4_q] = f(X + h*k3, U);
    X = X + h/6*(k1 + 2*k2 + 2*k3 + k4);
    Q = Q + h/6*(k1_q + 2*k2_q + 2*k3_q + k4_q);
end
F = casadi.Function('F', {X0, U}, {X, Q}, {'x0','p'}, {'xf', 'qf'});

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

% x_dagger = [x_opt(1:4); x_opt(5)-400]; % Disturbed state
xtilde_dagger = [-1500; 0; 0; 0; 0];

% [~, y_dagger, ~] = diesel_engine(x_dagger, u_opt, n_e, model);
% x_egr_dagger = y_dagger.x_egr;

% "Lift" initial conditions
Xk = casadi.MX.sym('X0', 5);
w = [w; Xk];
lbw = [lbw; xtilde_dagger];
ubw = [ubw; xtilde_dagger];
w0 = [w0; xtilde_dagger];

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
solver = casadi.nlpsol('solver', 'ipopt', nlp);

solution = solver('x0',  w0, ...      % Initial guess
                  'lbx', lbw, ...     % Lower variable bound
                  'ubx', ubw, ...     % Upper variable bound
                  'lbg', lbg, ...     % Lower constraint bound
                  'ubg', ubg);        % Upper constraint bound
              
w_opt = full(solution.x);

%% Extracting

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
clf
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
clf
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

% x_egr = [];
% lambda_O = [];
% for i = 0:N-1
%     X_opt = [W_opt(1 + i*8); W_opt(2 + i*8); W_opt(3 + i*8);...
%              W_opt(4 + i*8); W_opt(5 + i*8)];
%     U_opt = [W_opt(6 + i*8); W_opt(7 + i*8); W_opt(8 + i*8)];
%     
%     [~, sig, ~] = diesel_engine(X_opt, U_opt, n_e, model);
%     x_egr = [x_egr; sig.x_egr];
%     lambda_O = [lambda_O; sig.lambda_O];
% end

% figure(3)
% clf
% subplot(311)
% plot(tgrid, linspace(x_egr_sp, x_egr_sp, N+1), 'k--')
% hold on
% plot(tgrid(1:end-1), x_egr)
% title('x_{egr}')
% 
% subplot(312)
% plot(tgrid,  W_opt(2:8:end) - W_opt(1:8:end))
% title('p_{em} - p_{im}')
% 
% subplot(313)
% plot(tgrid, linspace(lambda_O_sp, lambda_O_sp, N+1), 'k--')
% hold on
% plot(tgrid(1:end-1), lambda_O)
% title('\lambda_{O}')
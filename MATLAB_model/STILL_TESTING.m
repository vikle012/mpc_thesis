clc
clear
load parameterData

% From engine_map.m
x_opt = [1.565301694037181e+05; 1.625445195919409e+05; 0.228497904007297; ...
         0.120767676163110; 6.598248922686063e+03];
u_opt = [1.109764478798882e+02; 15.365754977166260; 70.831108417661596];    
x_egr_sp = 0.026231899430965;       % Set-point
lambda_O_sp = 2.196994711101693;    % Set-point

p_amb = model.p_amb;
BSR_opt = model.BSR_opt;
PI_egropt = model.PI_egropt;

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
xtilde_dot = A(x_opt, u_opt)*xtilde + B(x_opt, u_opt)*utilde; % Linearized continous time system

T = 2;  % Time horizon
N = 20; % Prediction horizon/number of control intervals

q1 = 1;
q2 = 1;

% Objective function
L = xtilde'*q1*xtilde + utilde'*q2*utilde;
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

x_lbw = [0.5*p_amb; 0.5*p_amb;  0; 0; 100*pi/30]; 
x_ubw = [10*p_amb;  20*p_amb;   1; 1; 200000*pi/30];
u_lbw = [1; 0; 20];
u_ubw = [250; 100; 100];
x_dagger = [x_opt(1:4); x_opt(5)-400]; % Disturbed state

[~, y_dagger, ~] = diesel_engine(x_dagger, u_opt, n_e, model);
x_egr_dagger = y_dagger.x_egr;

% "Lift" initial conditions
Xk = casadi.MX.sym('X0', 5);
w = [w; Xk];
lbw = [lbw; x_dagger];
ubw = [ubw; x_dagger];
w0 = [w0; x_dagger];

% Formulate the NLP
for k=0:N-1    
    % New NLP variable for the control     
    Uk = casadi.MX.sym(['U_' num2str(k)], 3);
    w = [w; Uk];
    lbw = [lbw; u_lbw];
    ubw = [ubw; u_ubw]; 
    w0 = [w0;  u_opt];

    % Integrate till the end of the interval
    Fk = F('x0', Xk, 'p', Uk);
    Xk_next = Fk.xf;
    J = J + Fk.qf;
    
    [~, Yk, ~] = diesel_engine(Xk, Uk, n_e, model);
    
    % New NLP variable for state at end of interval
    Xk = casadi.MX.sym(['X_' num2str(k+1)], 5);
    w = [w; Xk];
    lbw = [lbw; x_lbw];
    ubw = [ubw; x_ubw];
    w0 = [w0; x_opt];

    % Add equality constraint
    G = [G; Xk_next-Xk];
    lbg = [lbg; zeros(5,1)];
    ubg = [ubg; zeros(5,1)];
    
    % Constraints from Simulink model
    M_e = Yk.M_e;
    G = [G; M_e];
    lbg = [lbg; 800];
    ubg = [ubg; inf];
    
    % From turbine.m
    PI_t = Yk.PI_t;          % PI_t <= 0.9999            "Saturation"
    BSR = Yk.BSR;            % 0 <= BSR <= 2*BSR_opt      Saturation
    eta_tm = Yk.eta_tm;      % 0.2 <= eta_tm <= 1         Saturation

    constr_t = [PI_t; BSR; eta_tm];
    constr_t_lbg = [0; 0; 0.2];
    constr_t_ubg = [0.9999; 2*BSR_opt; 1];

    % From EGR_system.m
    PI_egr = Yk.PI_egr;      % PI_egropt <= PI_egr <= 1   Saturation

    constr_EGR = PI_egr;
    constr_EGR_lbg = PI_egropt;
    constr_EGR_ubg = 1;

    % From cylinder.m
    lambda_air = Yk.lambda_air;  % 1.2 < lambda_air      Discussed value
    X_Oe = Yk.X_Oe;              % 0 <= X_Oe             "Saturation"

    constr_cyl = [lambda_air; X_Oe];
    constr_cyl_lbg = [1.2; 0];
    constr_cyl_ubg = [inf; inf;];

    % From compressor.m
    PI_c = Yk.PI_c;          % 1 <= PI_c                 "Saturation"
    W_c = Yk.W_c;            % 1e-4 <= W_c <= inf         Saturation
    eta_c = Yk.eta_c;        % 0.2 <= eta_c              "Saturation"

    constr_c = [PI_c; W_c; eta_c];
    constr_c_lbg = [1; 1e-4; 0.2];
    constr_c_ubg = [inf; inf; 1];

    constr = [constr_t; constr_EGR; constr_cyl; constr_c];
    constr_lbg = [constr_t_lbg; constr_EGR_lbg; constr_cyl_lbg; constr_c_lbg];
    constr_ubg = [constr_t_ubg; constr_EGR_ubg; constr_cyl_ubg; constr_c_ubg];
 
    % Add saturation constraints
    G = [G; constr];
    lbg = [lbg; constr_lbg];
    ubg = [ubg; constr_ubg];   
    
    % Add constraint for oxygen/fuel ratio
    G = [G; Yk.lambda_O];
    lbg = [lbg; lambda_O_sp];
    ubg = [ubg; inf]; 
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
W_opt = full(solution.x);

%% Plotting

tgrid = linspace(0, T, N+1);

figure(1)
clf
subplot(511)
plot(tgrid, linspace(x_opt(1), x_opt(1), N+1), 'k--')
hold on
plot(tgrid, W_opt(1:8:end))
title('p_{im}')

subplot(512)
plot(tgrid, linspace(x_opt(2), x_opt(2), N+1), 'k--')
hold on
plot(tgrid, W_opt(2:8:end))
title('p_{em}')

subplot(513)
plot(tgrid, linspace(x_opt(3), x_opt(3), N+1), 'k--')
hold on
plot(tgrid, W_opt(3:8:end))
title('X_{Oim}')

subplot(514)
plot(tgrid, linspace(x_opt(4), x_opt(4), N+1), 'k--')
hold on
plot(tgrid, W_opt(4:8:end))
title('X_{Oem}')

subplot(515)
plot(tgrid, linspace(x_opt(5), x_opt(5), N+1), 'k--')
hold on
plot(tgrid, W_opt(5:8:end))
title('\omega_t')

figure(2)
clf
subplot(311)
plot(tgrid, linspace(u_opt(1), u_opt(1), N+1), 'k--')
hold on
stairs(tgrid, [W_opt(6:8:end); nan])
title('u_\delta')

subplot(312)
plot(tgrid, linspace(u_opt(2), u_opt(2), N+1), 'k--')
hold on
stairs(tgrid, [W_opt(7:8:end); nan])
title('u_{egr}')

subplot(313)
plot(tgrid, linspace(u_opt(3), u_opt(3), N+1), 'k--')
hold on
stairs(tgrid, [W_opt(8:8:end); nan])
title('u_{vgt}')

x_egr = [];
lambda_O = [];
for i = 0:N-1
    X_opt = [W_opt(1 + i*8); W_opt(2 + i*8); W_opt(3 + i*8);...
             W_opt(4 + i*8); W_opt(5 + i*8)];
    U_opt = [W_opt(6 + i*8); W_opt(7 + i*8); W_opt(8 + i*8)];
    
    [~, sig, ~] = diesel_engine(X_opt, U_opt, n_e, model);
    x_egr = [x_egr; sig.x_egr];
    lambda_O = [lambda_O; sig.lambda_O];
end

figure(3)
clf
subplot(311)
plot(tgrid, linspace(x_egr_sp, x_egr_sp, N+1), 'k--')
hold on
plot(tgrid(1:end-1), x_egr)
title('x_{egr}')

subplot(312)
plot(tgrid,  W_opt(2:8:end) - W_opt(1:8:end))
title('p_{em} - p_{im}')

subplot(313)
plot(tgrid, linspace(lambda_O_sp, lambda_O_sp, N+1), 'k--')
hold on
plot(tgrid(1:end-1), lambda_O)
title('\lambda_{O}')
clc
clear
load parameterData

n_e = 1500;
x_opt = [1.565301694037181e+05; 1.625445195919409e+05; 0.228497904007297; ...
         0.120767676163110; 6.598248922686063e+03];

u_opt = [1.109764478798882e+02; 15.365754977166260; 70.831108417661596];    
     
x_egr_sp = 0.026231899430965;
lambda_O_sp = 2.196994711101693;

p_amb = model.p_amb;
BSR_opt = model.BSR_opt;
PI_egropt = model.PI_egropt;

% Linearized MPC with direct multiple shooting

X = casadi.MX.sym('X', 5);
U = casadi.MX.sym('U', 3);

[X_dot, Y, ~] = diesel_engine(X, U, n_e, model);

% Constraints from Simulink model
% From turbine.m
PI_t = Y.PI_t;          % PI_t <= 0.9999            "Saturation"
BSR = Y.BSR;            % 0 <= BSR <= 2*BSR_opt      Saturation
eta_tm = Y.eta_tm;      % 0.2 <= eta_tm <= 1         Saturation

constr_t = [PI_t; BSR; eta_tm];
constr_t_lbg = [0; 0; 0.2];
constr_t_ubg = [0.9999; 2*BSR_opt; 1];

% From EGR_system.m
PI_egr = Y.PI_egr;      % PI_egropt <= PI_egr <= 1   Saturation

constr_EGR = PI_egr;
constr_EGR_lbg = PI_egropt;
constr_EGR_ubg = 1;

% From cylinder.m
lambda_air = Y.lambda_air;  % 1.2 < lambda_air      Discussed value
X_Oe = Y.X_Oe;              % 0 <= X_Oe             "Saturation"

constr_cyl = [lambda_air; X_Oe];
constr_cyl_lbg = [1.2; 0];
constr_cyl_ubg = [inf; inf;];

% From compressor.m
PI_c = Y.PI_c;          % 1 <= PI_c                 "Saturation"
W_c = Y.W_c;            % 1e-4 <= W_c <= inf         Saturation
eta_c = Y.eta_c;        % 0.2 <= eta_c              "Saturation"

constr_c = [PI_c; W_c; eta_c];
constr_c_lbg = [1; 1e-4; 0.2];
constr_c_ubg = [inf; inf; 1];

constr = [constr_t; constr_EGR; constr_cyl; constr_c];
constr_lbg = [constr_t_lbg; constr_EGR_lbg; constr_cyl_lbg; constr_c_lbg];
constr_ubg = [constr_t_ubg; constr_EGR_ubg; constr_cyl_ubg; constr_c_ubg];

% Linearizing system around x_opt and u_opt as stationary points
%  follows the format in "Reglerteori"

A = jacobian(diesel_engine(X, U, n_e, model), X);
B = jacobian(diesel_engine(X, U, n_e, model), U);

Xtilde_dot = A*(X - x_opt) + B*(U - u_opt); % Linearized continous time system

T = 10; % Time horizon
N = 20; % Prediction horizon

Q1 = 1;
Q2 = 1;
Q3 = 1;

% Objective function
L = Q1*(Y.x_egr - x_egr_sp)^2 + Q2*(X(2) - X(1)) + U'*Q3*U;    %(u - uold)'*Q3*(u - uold);   % Objective term for one iteration

% Continuous time dynamics using diesel_engine.m
f = casadi.Function('f', {X, U}, {Xtilde_dot, L});

Ts = T/N; % Sample time
dt = Ts;
Qk = 0;   % Objective cost function
X0 = casadi.MX.sym('X0', 5);
X = X0;

% Formulate discrete time dynamics using RK4
for k=1:N 
    [k1, k1_q] = f(X, U);
    [k2, k2_q] = f(X + 0.5*dt*k1, U);
    [k3, k3_q] = f(X + 0.5*dt*k2, U);
    [k4, k4_q] = f(X + dt*k3, U);
    X = X + dt/6*(k1 + 2*k2 + 2*k3 + k4);
    Qk = Qk + dt/6*(k1_q + 2*k2_q + 2*k3_q + k4_q);
end
F = casadi.Function('F', {X0, U}, {X, Qk}, {'x0','p'}, {'xf', 'qf'});

% Start with an empty NLP
w   = {};
w0  = [];
lbw = [];
ubw = [];
J   = 0;
G   = {};
lbg = [];
ubg = [];

x_lbw = [0.5*p_amb; 0.5*p_amb;  0; 0; 100*pi/30]; 
x_ubw = [10*p_amb;  20*p_amb;   1; 1; 200000*pi/30];
u_lbw = [1; 0; 20];
u_ubw = [250; 100; 100];

x_dagger = [x_opt(1:4); x_opt(5)-300];

% "Lift" initial conditions
Xk = casadi.MX.sym('X0', 5);
w = {w{:}, Xk};
lbw = [lbw; x_dagger];
ubw = [ubw; x_dagger];
w0 = [w0; x_dagger];

% Formulate the NLP
for k=0:N-1
    % New NLP variable for the control
    Uk = casadi.MX.sym(['U_' num2str(k)], 3);
    w = {w{:}, Uk};
    lbw = [lbw; u_lbw];
    ubw = [ubw; u_ubw];
    w0 = [w0;  u_opt]; %<--------------------- endast f?rsta g?ngen?

    % Integrate till the end of the interval
    Fk = F('x0', Xk, 'p', Uk);
    Xk_end = Fk.xf;
    J = J + Fk.qf;
    
    [~, Yk, ~] = diesel_engine(Xk, Uk, n_e, model);
    
    % New NLP variable for state at end of interval
    Xk = casadi.MX.sym(['X_' num2str(k+1)], 5);
    w = [w, {Xk}];
    lbw = [lbw; x_lbw];
    ubw = [ubw; x_ubw];
    w0 = [w0; x_opt]; %<---------------------- Xk(i-1)?

    % Add equality constraint
    G = [G, {Xk_end-Xk}];
    lbg = [lbg; zeros(5,1)];
    ubg = [ubg; zeros(5,1)];
    
    % Add saturation constraints
%     g = [g, {constr}];
%     lbg = [lbg; constr_lbg];
%     ubg = [ubg; constr_ubg];
%     
    
    % Add constraint for oxygen/fuel ratio
%     G = [G, {Yk.lambda_O}];
%     lbg = [lbg; lambda_O_sp];
%     ubg = [ubg; inf]; 

end

clc

% Create an NLP solver function
nlp = struct('f', J, 'x', vertcat(w{:}), 'g', vertcat(G{:}));
solver = casadi.nlpsol('solver', 'ipopt', nlp);

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
plot(tgrid, linspace(x_opt(1), x_opt(1), N+1), 'k--', 'LineWidth',2)
hold on
plot(tgrid, W_opt(1:8:end))
title('p_{im}')

subplot(512)
plot(tgrid, linspace(x_opt(2), x_opt(2), N+1), 'k--', 'LineWidth',2)
hold on
plot(tgrid, W_opt(2:8:end))
title('p_{em}')

subplot(513)
plot(tgrid, linspace(x_opt(3), x_opt(3), N+1), 'k--', 'LineWidth',2)
hold on
plot(tgrid, W_opt(3:8:end))
title('X_{Oim}')

subplot(514)
plot(tgrid, linspace(x_opt(4), x_opt(4), N+1), 'k--', 'LineWidth',2)
hold on
plot(tgrid, W_opt(4:8:end))
title('X_{Oem}')

subplot(515)
plot(tgrid, linspace(x_opt(5), x_opt(5), N+1), 'k--', 'LineWidth',2)
hold on
plot(tgrid, W_opt(5:8:end))
title('\omega_t')

% figure(2)
% clf
% subplot(311)
% plot(tgrid, linspace(u_opt(1), u_opt(1), N+1), 'k--', 'LineWidth',2)
% hold on
% % stairs(tgrid, W_opt(6:8:end))
% title('u_\delta')
% 
% subplot(312)
% plot(tgrid, linspace(u_opt(2), u_opt(2), N+1), 'k--', 'LineWidth',2)
% hold on
% % stairs(tgrid, W_opt(7:8:end))
% title('u_{egr}')
% 
% subplot(313)
% plot(tgrid, linspace(u_opt(3), u_opt(3), N+1), 'k--', 'LineWidth',2)
% hold on
% % stairs(tgrid, W_opt(8:8:end))
% title('u_{vgt}')
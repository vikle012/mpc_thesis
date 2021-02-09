clear
clc

addpath('C:\Users\Jonte\Documents\GitHub\mpc_thesis\MATLAB_model')
addpath('C:\Users\Jonte\Documents\GitHub\mpc_thesis\Controllers')
addpath('C:\Users\Jonte\Documents\GitHub\mpc_thesis\WHTC')
addpath('C:\Users\Jonte\Documents\GitHub\mpc_thesis\WahlstromErikssonTCDI_EGR_VGT')
addpath('C:\Users\Jonte\Documents\GitHub\mpc_thesis\CasADi\casadi-windows-matlabR2016a-v3.5.5')
load parameterData

M_e_input = [40; 80]; % Normalized values
n_e_input = [1200; 1200];
[x_ref, u_ref] = reference_generator(M_e_input, n_e_input);

time_vector = [0 0.1]';

control.u_egract = 0; % without EGR-actuator dynamics
control.u_vgtact = 0; % without VGT-actuator dynamics

%% Select Controller

controller_object = 'SL_ref';

integral_action = 1;
% 0: without integral action
% 1: with integral action

simulation_time = 10;

%% Tuning Parameters

% Parameters required in set-up
global init_param;
init_param.model = model;
init_param.integral_action = integral_action;
init_param.T_s = 0.01;
init_param.N = 40;

q1 = 1*100/((0.5*model.p_amb + 10*model.p_amb)/2)^2;
q2 = 1*100/((0.5*model.p_amb + 20*model.p_amb)/2)^2;
q3 = 50/((0 + 1)/2)^2;
q4 = 50/((0 + 1)/2)^2;
q5 = 1*100/((100*pi/30 + 200000*pi/30)/2)^2;

r1 = 5/((1 + 250)/2)^2;
r2 = 5/((0 + 100)/2)^2;
r3 = 5/((20 + 100)/2)^2; 

% Integral action
s1 = 100/((1 + 250)/2)^2;
s2 = 100/((0 + 100)/2)^2;
s3 = 100/((20 + 100)/2)^2; 

init_param.Q = diag([q1 q2 q3 q4 q5]);
init_param.R = diag([r1 r2 r3]);
init_param.S = diag([s1 s2 s3]);

init_param.u_old = u_ref(:,1); % Not happy with this fix, but works

%% Simulation Set-up

simulate.T_s = init_param.T_s;

if or(strcmp(controller_object, "SLpreview_ref"), strcmp(controller_object, "SLpreview_current"))
    % Not proud of this solution
    x_ref_all = [];
    u_ref_all = [];
    for i = 1:length(time_vector)
        t1 = time_vector(i);
        if i == length(time_vector)
            t2 = simulation_time;
            repeat = (t2 - t1)/simulate.T_s + 1;
        else
            t2 = time_vector(i+1);
            repeat = (t2 - t1)/simulate.T_s;
        end
        x_ref_all = [x_ref_all repmat(x_ref(:,i), 1, repeat)];
        u_ref_all = [u_ref_all repmat(u_ref(:,i), 1, repeat)];
    end
    % Extend further
    x_ref_all = [x_ref_all repmat(x_ref(:,end), 1, init_param.N)];
    u_ref_all = [u_ref_all repmat(u_ref(:,end), 1, init_param.N)];
    simulate.n_e = timeseries(n_e_input, time_vector); % Row vector since "From Workspace"
    simulate.x_ref = timeseries(x_ref_all, 0);
    simulate.u_ref = timeseries(u_ref_all, 0);
else
    simulate.x_ref = timeseries(x_ref, time_vector); % Row vector since "From Workspace"
    simulate.u_ref = timeseries(u_ref, time_vector);
    simulate.n_e = timeseries(n_e_input, time_vector);
end

% --- Simulink initialization ---
simulate.p_im_Init          = x_ref(1,1);
simulate.p_em_Init          = x_ref(2,1);
simulate.X_Oim_Init         = x_ref(3,1);
simulate.X_Oem_Init         = x_ref(4,1);
simulate.omega_t_Init       = x_ref(5,1);
simulate.utilde_egr1_Init   = 0;
simulate.utilde_egr2_Init   = 0;
simulate.utilde_vgt_Init    = 0;

% Not used
simulate.u_egr_Init = u_ref(2,1);
simulate.u_vgt_Init = u_ref(3,1);

[~,y,~] = diesel_engine(x_ref(:,1), u_ref(:,1), n_e_input(1), model);
simulate.x_r_Init = y.x_r;
simulate.T_1_Init = y.T_1;
% -------------------------------

% Open system
open_system('controlledSystem')

% Simulink controller set-up
set_param('controlledSystem/MPC/MATLAB System', 'System', ...
    controller_object, 'SimulateUsing', 'Interpreted execution');
set_param('controlledSystem', 'SimulationCommand', 'Update')

% Simulation
sim('controlledSystem', simulation_time)

%% Plotting

x_data = [simp_im, simp_em, simX_Oim, simX_Oem, simn_t*pi/30];
x_titles = ["p_{im} [Pa]", "p_{em} [Pa]", "X_{Oim} [-]", "X_{Oem} [-]", "\omega_t [rad/s]"];

figure(1)
for i = 1:5
    subplot(5,1,i)
    hold on
    plot(simTime, x_data(:,i))
    stairs([time_vector; simulation_time], [x_ref(i,1); x_ref(i,2); x_ref(i,2)], 'k:');
    ylabel(x_titles(i))
end
xlabel('Time [s]')

u_delta = squeeze(simu_delta);
u_egr = squeeze(simu_egr);
u_vgt = squeeze(simu_vgt);

u_data = [u_delta, u_egr, u_vgt];
u_titles = ["u_\delta [mg/cycle]", "u_{egr} [%]", "u_{vgt} [%]"];

figure(2)
for i = 1:3
    subplot(3,1,i)
    hold on
    stairs(simTime, u_data(:,i))
    stairs([time_vector; simulation_time], [u_ref(i,1); u_ref(i,2); u_ref(i,2)], 'k:');
    ylabel(u_titles(i))
end
xlabel('Time [s]')

M_e = squeeze(simM_e);
lambda_O = squeeze(simlambda);

figure(3)
hold on
plot(simTime, M_e)
% stairs([time_vector; simulation_time], [M_e_input; M_e_input(end)], 'k:');
ylabel('M_e [Nm]')
xlabel('Time [s]')

figure(4)
hold on
plot(simTime, lambda_O)
ylabel('\lambda_O [-]')
xlabel('Time [s]')
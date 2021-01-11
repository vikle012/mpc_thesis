clear
clc

addpath('C:\Users\Jonte\Documents\GitHub\mpc_thesis\MATLAB_model')
addpath('C:\Users\Jonte\Documents\GitHub\mpc_thesis\Controllers')
addpath('C:\Users\Jonte\Documents\GitHub\mpc_thesis\WahlstromErikssonTCDI_EGR_VGT')
addpath('C:\Users\Jonte\Documents\GitHub\mpc_thesis\CasADi\casadi-windows-matlabR2016a-v3.5.5')
load parameterData

M_e_input = [800; 900];
n_e_input = [1500; 1500];
[x_ref, u_ref] = find_trajectory(M_e_input, n_e_input, model);

time_vector = [0 0.1]';
simulation_time = 2;

%%
% figure(1)
% for i = 1:5
%    subplot(5,1,i)
%    plot(time, x_ref(i,:))
% end
% 
% figure(2)
% for i = 1:3
%    subplot(3,1,i)
%    plot(time, u_ref(i,:))
% end

%%
global init_param;

control.u_egract = 0;
% 1: with EGR-actuator dynamics
% 0: without EGR-actuator dynamics

control.u_vgtact = 0;
% 1: with VGT-actuator dynamics
% 0: without VGT-actuator dynamics

init_param.n_e = n_e_input(1); % n_e_init
init_param.model = model;

%% Select Controller and Tune Parameters

controller_object = 'linReferenceEach';

init_param.T_s = 0.01;
init_param.N = 40;

q1 = 100/((0.5*model.p_amb + 10*model.p_amb)/2)^2;
q2 = 100/((0.5*model.p_amb + 20*model.p_amb)/2)^2;
q3 = 50/((0 + 1)/2)^2;
q4 = 50/((0 + 1)/2)^2;
q5 = 100/((100*pi/30 + 200000*pi/30)/2)^2;

r1 = 200/((1 + 250)/2)^2;
r2 = 0.005/((0 + 100)/2)^2;
r3 = 0.005/((20 + 100)/2)^2; 

init_param.Q = diag([q1 q2 q3 q4 q5]);
init_param.R = diag([r1 r2 r3]);

init_param.u_old = u_ref(:,1); % Not happy with this fix, but works

%% Simulation
simulate.T_s = init_param.T_s;
simulate.n_e = timeseries(n_e_input, time_vector); % Row vector since "From Workspace"
simulate.x_ref = timeseries(x_ref, time_vector);
simulate.u_ref = timeseries(u_ref, time_vector);

% --- Simulink initialization ---
simulate.p_im_Init          = simulate.x_ref.data(1,:,1);
simulate.p_em_Init          = simulate.x_ref.data(2,:,1);
simulate.X_Oim_Init         = simulate.x_ref.data(3,:,1);
simulate.X_Oem_Init         = simulate.x_ref.data(4,:,1);
simulate.omega_t_Init       = simulate.x_ref.data(5,:,1);
simulate.utilde_egr1_Init   = 0;
simulate.utilde_egr2_Init   = 0;
simulate.utilde_vgt_Init    = 0;

simulate.u_egr_Init = simulate.u_ref.data(1,:,1);
simulate.u_vgt_Init = simulate.u_ref.data(2,:,1);

[~,y,~] = diesel_engine(simulate.x_ref.data(:,:,1), ...
                        simulate.u_ref.data(:,:,1), init_param.n_e, model);
simulate.x_r_Init = y.x_r;
simulate.T_1_Init = y.T_1;
% -------------------------------

set_param('controlledSystem/MPC/MATLAB System', ...
          'System', controller_object, ... % Change controller object here
          'SimulateUsing', 'Interpreted execution');
% save_system('controlledSystem/MPC/MATLAB System');
set_param('controlledSystem', 'SimulationCommand', 'Update')
% Simulation
sim('controlledSystem', simulation_time)

%% Plotting

x_data = [simp_im, simp_em, simX_Oim, simX_Oem, simn_t*pi/30];
x_titles = ["p_{im}", "p_{em}", "X_{Oim}", "X_{Oem}", "\omega_t"];

figure(1)
for i = 1:5
    subplot(5,1,i)
    hold on
    plot(tout, x_data(:,i))
    stairs([time_vector; simulation_time], [x_ref(i,1); x_ref(i,2); x_ref(i,2)], 'k:');
    title(x_titles(i))
end

u_delta = [];
u_egr = [];
u_vgt = [];
for i = 1:(simulation_time/init_param.T_s + 1)
   u_delta = [u_delta; simu_delta(:,:,i)];
   u_egr = [u_egr; simu_egr(:,:,i)];
   u_vgt = [u_vgt; simu_vgt(:,:,i)];
end
u_data = [u_delta, u_egr, u_vgt];
u_titles = ["u_\delta", "u_{egr}", "u_{vgt}"];

figure(2)
for i = 1:3
    subplot(3,1,i)
    hold on
    stairs(linspace(0, simulation_time, simulation_time/init_param.T_s + 1), u_data(:,i))
    stairs([time_vector; simulation_time], [u_ref(i,1); u_ref(i,2); u_ref(i,2)], 'k:');
    title(u_titles(i))
end

M_e = [];
lambda_O = [];
for i = 1:length(tout)
   M_e = [M_e; simM_e(:,:,i)];
   lambda_O = [lambda_O; simlambda(:,:,i)];
end

figure(3)
hold on
plot(tout, M_e)
stairs([time_vector; simulation_time], [M_e_input; M_e_input(end)], 'k:');
title('M_e')

figure(4)
hold on
plot(tout, lambda_O)
title('\lambda_O')
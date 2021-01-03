clear
clc

addpath('C:\Users\Jonte\Documents\GitHub\mpc_thesis\MATLAB_model')
addpath('C:\Users\Jonte\Documents\GitHub\mpc_thesis\Controllers')
addpath('C:\Users\Jonte\Documents\GitHub\mpc_thesis\WahlstromErikssonTCDI_EGR_VGT')
addpath('C:\Users\Jonte\Documents\GitHub\mpc_thesis\CasADi\casadi-windows-matlabR2016a-v3.5.5')
load parameterData


% max_torque = 2220;
% max_speed = 2000;
% 
% load WHTC_data

% M_e = max_torque * torque/100;
% n_e = max_speed * speed/100;

% subplot(2,1,1)
% plot(time, M_e)
% subplot(2,1,2)
% plot(time, n_e)

% [x_ref, u_ref] = find_trajectory(M_e, n_e, model);
% 
% %%
% 
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

global tuning_param;

control.u_egract = 0;
% 1: with EGR-actuator dynamics
% 0: without EGR-actuator dynamics

control.u_vgtact = 0;
% 1: with VGT-actuator dynamics
% 0: without VGT-actuator dynamics

% From engine_map.m with m = 800
x_800 = [156530.169403718; 162544.519591941; 0.228497904007297; ...
         0.120767676163110; 6598.24892268606];
u_800 = [110.976447879888; 15.3657549771663; 70.8311084176616];  

% From engine_map.m with m = 900
x_900 = [163715.168088435; 167962.252867181; 0.229672106525997; ...
         0.116324617241729; 6897.33534814885];
u_900 = [122.033437081754; 12.7268611588264; 74.8549539702291];

n_e = 1500;
tuning_param.n_e = n_e; % n_e_init

%% Tuning Parameters
tuning_param.T_s = 0.05/5;
tuning_param.N = 40;

q1 = 100/((0.5*model.p_amb + 10*model.p_amb)/2)^2;
q2 = 100/((0.5*model.p_amb + 20*model.p_amb)/2)^2;
q3 = 70/((0 + 1)/2)^2;
q4 = 50/((0 + 1)/2)^2;
q5 = 50/((100*pi/30 + 200000*pi/30)/2)^2;

r1 = 50/((1 + 250)/2)^2;
r2 = 0.005/((0 + 100)/2)^2;
r3 = 0.005/((20 + 100)/2)^2;

tuning_param.Q = diag([q1 q2 q3 q4 q5]);
tuning_param.R = diag([r1 r2 r3]);

%% Simulation
simulation_time = 1;
time_vector = [0 0.1]';

% Note: 500 <= n_e <= 2000
simulate.n_e = timeseries(n_e, time_vector); % Row vector since "From Workspace"
simulate.x_ref = timeseries([x_800'; x_900'], time_vector);
simulate.u_ref = timeseries([u_800'; u_900'], time_vector);

% --- Simulink initialization ---
simulate.p_im_Init          = simulate.x_ref.data(1,1);
simulate.p_em_Init          = simulate.x_ref.data(1,2);
simulate.X_Oim_Init         = simulate.x_ref.data(1,3);
simulate.X_Oem_Init         = simulate.x_ref.data(1,4);
simulate.omega_t_Init       = simulate.x_ref.data(1,5);
simulate.utilde_egr1_Init   = 0;
simulate.utilde_egr2_Init   = 0;
simulate.utilde_vgt_Init    = 0;

simulate.u_egr_Init = simulate.u_ref.data(1,1);
simulate.u_vgt_Init = simulate.u_ref.data(1,2);

[~,y,~] = ...
    diesel_engine(simulate.x_ref.data(1,:), simulate.u_ref.data(1,:), ...
                  tuning_param.n_e(1), model);
simulate.x_r_Init = y.x_r;
simulate.T_1_Init = y.T_1;
% -------------------------------

% Simulation
sim('each_model', simulation_time)

%% Plotting --- Kolla ?ver

figure(1)
subplot(511)
plot(tout, linspace(x_800(1), x_800(1), length(tout)), 'k-.', 'Linewidth', 1.3)
hold on
plot(tout, linspace(x_900(1), x_900(1), length(tout)), 'k:', 'Linewidth', 1.3)
% plot(tout, linspace(x_lbw(1), x_lbw(1), length(tout)), 'r--')
% plot(tout, linspace(x_ubw(1), x_ubw(1), length(tout)), 'r--')
plot(tout, simp_im)
title('p_{im}')

subplot(512)
plot(tout, linspace(x_800(2), x_800(2), length(tout)), 'k-.', 'Linewidth', 1.3)
hold on
plot(tout, linspace(x_900(2), x_900(2), length(tout)), 'k:', 'Linewidth', 1.3)
plot(tout, simp_em)
% plot(tout, linspace(x_lbw(2), x_lbw(2), length(tout)), 'r--')
% plot(tout, linspace(x_ubw(2), x_ubw(2), length(tout)), 'r--')
title('p_{em}')

subplot(513)
plot(tout, linspace(x_800(3), x_800(3), length(tout)), 'k-.', 'Linewidth', 1.3)
hold on
plot(tout, linspace(x_900(3), x_900(3), length(tout)), 'k:', 'Linewidth', 1.3)
plot(tout, simX_Oim)
% plot(tout, linspace(x_lbw(3), x_lbw(3), length(tout)), 'r--')
% plot(tout, linspace(x_ubw(3), x_ubw(3), length(tout)), 'r--')
title('X_{Oim}')

subplot(514)
plot(tout, linspace(x_800(4), x_800(4), length(tout)), 'k-.', 'Linewidth', 1.3)
hold on
plot(tout, linspace(x_900(4), x_900(4), length(tout)), 'k:', 'Linewidth', 1.3)
plot(tout, simX_Oem)
% plot(tout, linspace(x_lbw(4), x_lbw(4), length(tout)), 'r--')
% plot(tout, linspace(x_ubw(4), x_ubw(4), length(tout)), 'r--')
title('X_{Oem}')

subplot(515)
plot(tout, linspace(x_800(5), x_800(5), length(tout)), 'k-.', 'Linewidth', 1.3)
hold on
plot(tout, linspace(x_900(5), x_900(5), length(tout)), 'k:', 'Linewidth', 1.3)
plot(tout, simn_t*pi/30)
% plot(tout, linspace(x_lbw(5), x_lbw(5), length(tout)), 'r--')
% plot(tout, linspace(x_ubw(5), x_ubw(5), length(tout)), 'r--')
title('\omega_t')

u_delta = [];
u_egr = [];
u_vgt = [];
for i = 1:(simulation_time/tuning_param.T_s + 1)
   u_delta = [u_delta; simu_delta(:,:,i)];
   u_egr = [u_egr; simu_egr(:,:,i)];
   u_vgt = [u_vgt; simu_vgt(:,:,i)];

end

figure(2)
subplot(311)
plot(tout, linspace(u_800(1), u_800(1), length(tout)), 'k-.', 'Linewidth', 1.3)
hold on
plot(tout, linspace(u_900(1), u_900(1), length(tout)), 'k:', 'Linewidth', 1.3)
stairs(linspace(0, simulation_time, simulation_time/tuning_param.T_s + 1), u_delta)
% plot(tout, linspace(u_lbw(1), u_lbw(1), length(tout)), 'r--')
% plot(tout, linspace(u_ubw(1), u_ubw(1), length(tout)), 'r--')
title('u_\delta')

subplot(312)
plot(tout, linspace(u_800(2), u_800(2), length(tout)), 'k-.', 'Linewidth', 1.3)
hold on
plot(tout, linspace(u_900(2), u_900(2), length(tout)), 'k:', 'Linewidth', 1.3)
stairs(linspace(0, simulation_time, simulation_time/tuning_param.T_s + 1), u_egr)
% plot(tout, linspace(u_lbw(2), u_lbw(2), length(tout)), 'r--')
% plot(tout, linspace(u_ubw(2), u_ubw(2), length(tout)), 'r--')
title('u_{egr}')

subplot(313)
plot(tout, linspace(u_800(3), u_800(3), length(tout)), 'k-.', 'Linewidth', 1.3)
hold on
plot(tout, linspace(u_900(3), u_900(3), length(tout)), 'k:', 'Linewidth', 1.3)
stairs(linspace(0, simulation_time, simulation_time/tuning_param.T_s + 1), u_vgt)
% plot(tout, linspace(u_lbw(3), u_lbw(3), length(tout)), 'r--')
% plot(tout, linspace(u_ubw(3), u_ubw(3), length(tout)), 'r--')
title('u_{vgt}')

%%
M_e = [];
lambda_O = [];
for i = 1:length(tout)
   M_e = [M_e; simM_e(:,:,i)];
end

figure(3)
plot(tout, linspace(800, 800, length(tout)), 'k-.', 'Linewidth', 1.3)
hold on
plot(tout, linspace(900, 900, length(tout)), 'k:', 'Linewidth', 1.3)
plot(tout, M_e)
title('M_e')


clear
clc

addpath('C:\Users\Jonte\Documents\GitHub\mpc_thesis\MATLAB_model')
addpath('C:\Users\Jonte\Documents\GitHub\mpc_thesis\Controllers')
addpath('C:\Users\Jonte\Documents\GitHub\mpc_thesis\WahlstromErikssonTCDI_EGR_VGT')
addpath('C:\Users\Jonte\Documents\GitHub\mpc_thesis\CasADi\casadi-windows-matlabR2016a-v3.5.5')
load parameterData

control.u_egract = 0;
% 1: with EGR-actuator dynamics
% 0: without EGR-actuator dynamics

control.u_vgtact = 0;
% 1: with VGT-actuator dynamics
% 0: without VGT-actuator dynamics

control.disturbance = 1;
% 1: with disturbance
% 0: without disturbance
control.step_at = 1;

% From engine_map.m with m = 800
x_opt = [156530.169403718; 162544.519591941; 0.228497904007297; ...
                  0.120767676163110; 6598.24892268606];
u_opt = [110.976447879888; 15.3657549771663; 70.8311084176616];  

% From engine_map.m with m = 1200
x_step = [163715.168088435; 167962.252867181; 0.229672106525997; ...
         0.116324617241729; 6897.33534814885];
u_step = [122.033437081754; 12.7268611588264; 74.8549539702291];

simulate.n_e = [0 1500];
simulate.T_s = 0.2;
simulation_time = 5;

% Initialization
% Note: change in casadi_MPC.m aswell
simulate.p_im_Init          = x_opt(1);
simulate.p_em_Init          = x_opt(2);
simulate.X_Oim_Init         = x_opt(3);
simulate.X_Oem_Init         = x_opt(4);
simulate.omega_t_Init       = x_opt(5);
simulate.utilde_egr1_Init   = 0;
simulate.utilde_egr2_Init   = 0;
simulate.utilde_vgt_Init    = 0;

% Simulation
sim('gsMPC_model',simulation_time)

%% For plotting

x_lbw = [0.5*model.p_amb; 0.5*model.p_amb;  0; 0; 100*pi/30]; 
x_ubw = [10*model.p_amb;  20*model.p_amb;   1; 1; 200000*pi/30];
u_lbw = [1; 0; 20];
u_ubw = [250; 100; 100];
xtilde_lbw = x_lbw - x_opt; 
xtilde_ubw = x_ubw - x_opt;
utilde_lbw = u_lbw - u_opt;
utilde_ubw = u_ubw - u_opt;

%% Plotting

figure(1)
subplot(511)
plot(tout, linspace(x_opt(1), x_opt(1), length(tout)), 'k--', 'Linewidth', 1.3)
hold on
plot(tout, linspace(x_step(1), x_step(1), length(tout)), 'g--', 'Linewidth', 1.3)
% plot(tout, linspace(x_lbw(1), x_lbw(1), length(tout)), 'r--')
% plot(tout, linspace(x_ubw(1), x_ubw(1), length(tout)), 'r--')
plot(tout, simp_im)
title('p_{im}')

subplot(512)
plot(tout, linspace(x_opt(2), x_opt(2), length(tout)), 'k--', 'Linewidth', 1.3)
hold on
plot(tout, linspace(x_step(2), x_step(2), length(tout)), 'g--', 'Linewidth', 1.3)
plot(tout, simp_em)
% plot(tout, linspace(x_lbw(2), x_lbw(2), length(tout)), 'r--')
% plot(tout, linspace(x_ubw(2), x_ubw(2), length(tout)), 'r--')
title('p_{em}')

subplot(513)
plot(tout, linspace(x_opt(3), x_opt(3), length(tout)), 'k--', 'Linewidth', 1.3)
hold on
plot(tout, linspace(x_step(3), x_step(3), length(tout)), 'g--', 'Linewidth', 1.3)
plot(tout, simX_Oim)
% plot(tout, linspace(x_lbw(3), x_lbw(3), length(tout)), 'r--')
% plot(tout, linspace(x_ubw(3), x_ubw(3), length(tout)), 'r--')
title('X_{Oim}')

subplot(514)
plot(tout, linspace(x_opt(4), x_opt(4), length(tout)), 'k--', 'Linewidth', 1.3)
hold on
plot(tout, linspace(x_step(4), x_step(4), length(tout)), 'g--', 'Linewidth', 1.3)
plot(tout, simX_Oem)
% plot(tout, linspace(x_lbw(4), x_lbw(4), length(tout)), 'r--')
% plot(tout, linspace(x_ubw(4), x_ubw(4), length(tout)), 'r--')
title('X_{Oem}')

subplot(515)
plot(tout, linspace(x_opt(5), x_opt(5), length(tout)), 'k--', 'Linewidth', 1.3)
hold on
plot(tout, linspace(x_step(5), x_step(5), length(tout)), 'g--', 'Linewidth', 1.3)
plot(tout, simn_t*pi/30)
% plot(tout, linspace(x_lbw(5), x_lbw(5), length(tout)), 'r--')
% plot(tout, linspace(x_ubw(5), x_ubw(5), length(tout)), 'r--')
title('\omega_t')

u_delta = [];
u_egr = [];
u_vgt = [];
for i = 1:(simulation_time/simulate.T_s + 1)
   u_delta = [u_delta; simu_delta(:,:,i)];
   u_egr = [u_egr; simu_egr(:,:,i)];
   u_vgt = [u_vgt; simu_vgt(:,:,i)];

end

figure(2)
subplot(311)
plot(tout, linspace(u_opt(1), u_opt(1), length(tout)), 'k--', 'Linewidth', 1.3)
hold on
plot(tout, linspace(u_step(1), u_step(1), length(tout)), 'g--', 'Linewidth', 1.3)
stairs(linspace(0, simulation_time, simulation_time/simulate.T_s + 1), u_delta)
% plot(tout, linspace(u_lbw(1), u_lbw(1), length(tout)), 'r--')
% plot(tout, linspace(u_ubw(1), u_ubw(1), length(tout)), 'r--')
title('u_\delta')

subplot(312)
plot(tout, linspace(u_opt(2), u_opt(2), length(tout)), 'k--', 'Linewidth', 1.3)
hold on
plot(tout, linspace(u_step(2), u_step(2), length(tout)), 'g--', 'Linewidth', 1.3)
stairs(linspace(0, simulation_time, simulation_time/simulate.T_s + 1), u_egr)
% plot(tout, linspace(u_lbw(2), u_lbw(2), length(tout)), 'r--')
% plot(tout, linspace(u_ubw(2), u_ubw(2), length(tout)), 'r--')
title('u_{egr}')

subplot(313)
plot(tout, linspace(u_opt(3), u_opt(3), length(tout)), 'k--', 'Linewidth', 1.3)
hold on
plot(tout, linspace(u_step(3), u_step(3), length(tout)), 'g--', 'Linewidth', 1.3)
stairs(linspace(0, simulation_time, simulation_time/simulate.T_s + 1), u_vgt)
% plot(tout, linspace(u_lbw(3), u_lbw(3), length(tout)), 'r--')
% plot(tout, linspace(u_ubw(3), u_ubw(3), length(tout)), 'r--')
title('u_{vgt}')

%%
M_e = [];
lambda_O = [];
for i = 1:length(tout)
   M_e = [M_e; simM_e(:,:,i)];
   lambda_O = [lambda_O; simlambda(:,:,i)];
end

figure(3)
plot(tout, linspace(800, 800, length(tout)), 'k--', 'Linewidth', 1.3)
hold on
plot(tout, linspace(900, 900, length(tout)), 'g--', 'Linewidth', 1.3)
plot(tout, M_e)
title('M_e')

% figure(4)
% plot(tout, simx_egr)
% title('x_{egr}')
% 
% figure(5)
% plot(tout, simp_em-simp_im)
% title('p_{em} - p_{im}')
% 
% figure(6)
% plot(tout, lambda_O)
% title('\lambda_O')
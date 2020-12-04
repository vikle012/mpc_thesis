addpath('C:\Users\Jonte\Documents\GitHub\mpc_thesis\MATLAB_model')
addpath('C:\Users\Jonte\Documents\GitHub\mpc_thesis\Controllers')
addpath('C:\Users\Jonte\Documents\GitHub\mpc_thesis\WahlstromErikssonTCDI_EGR_VGT')
addpath('C:\Users\Jonte\Documents\GitHub\mpc_thesis\CasADi\casadi-windows-matlabR2016a-v3.5.5')
load parameterData

clc

control.u_egract=0;
% 1: with EGR-actuator dynamics
% 0: without EGR-actuator dynamics

control.u_vgtact=0;
% 1: with VGT-actuator dynamics
% 0: without VGT-actuator dynamics

x_opt = [156530.169403718; 162544.519591941; 0.228497904007297; ...
                  0.120767676163110; 6598.24892268606];
u_opt = [110.976447879888; 15.3657549771663; 70.8311084176616];  
x_step = [190976.306096801; 190976.304195917; 0.231400000000089; ...
          0.107150994106105; 7865.28279123926];
 
simulate.n_e = [0 1500];
simulate.T_s = 0.2;
simulation_time = 5;

% Initialization
simulate.p_im_Init          = x_step(1);
simulate.p_em_Init          = x_step(2);
simulate.X_Oim_Init         = x_step(3);
simulate.X_Oem_Init         = x_step(4);
simulate.omega_t_Init       = x_step(5);
simulate.utilde_egr1_Init   = 0;
simulate.utilde_egr2_Init   = 0;
simulate.utilde_vgt_Init    = 0;

% Simulation
sim('MPC_EGR_VGT',simulation_time)

%% Plotting

figure(1)
subplot(511)
plot(tout, linspace(x_opt(1), x_opt(1), length(tout)), 'k--', 'Linewidth', 1.3)
hold on
plot(tout, simp_im)
title('p_{im}')

subplot(512)
plot(tout, linspace(x_opt(2), x_opt(2), length(tout)), 'k--', 'Linewidth', 1.3)
hold on
plot(tout, simp_em)
title('p_{em}')

subplot(513)
plot(tout, linspace(x_opt(3), x_opt(3), length(tout)), 'k--', 'Linewidth', 1.3)
hold on
plot(tout, simX_Oim)
title('X_{Oim}')

subplot(514)
plot(tout, linspace(x_opt(4), x_opt(4), length(tout)), 'k--', 'Linewidth', 1.3)
hold on
plot(tout, simX_Oem)
title('X_{Oem}')

subplot(515)
plot(tout, linspace(x_opt(5), x_opt(5), length(tout)), 'k--', 'Linewidth', 1.3)
hold on
plot(tout, simn_t*pi/30)
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
hold on
stairs(linspace(0, simulation_time, simulation_time/simulate.T_s + 1), u_delta)
title('u_\delta')

subplot(312)
hold on
stairs(linspace(0, simulation_time, simulation_time/simulate.T_s + 1), u_egr)
title('u_{egr}')

subplot(313)
hold on
stairs(linspace(0, simulation_time, simulation_time/simulate.T_s + 1), u_vgt)
title('u_{vgt}')
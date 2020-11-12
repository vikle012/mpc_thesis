clc
clear

addpath('WahlstromErikssonTCDI_EGR_VGT')
addpath('MATLAB_model')
load parameterData

%% First simulation until reaching stationary point

control.u_egract=0;
% 1: with EGR-actuator dynamics
% 0: without EGR-actuator dynamics

control.u_vgtact=0;
% 1: with VGT-actuator dynamics
% 0: without VGT-actuator dynamics

%set initial values for the inputs
%First column: time vector
%Second column: data vector
simU.n_e=[0 1500]; 
simU.u_delta=[0 110];
simU.u_egr=[0 80];        
simU.u_vgt=[0 30];

%simulate the engine for the initial values until stationary point reached
sim('TCDI_EGR_VGT',20)

model.x_r_Init=simx_r(end);
model.T_1_Init=simT_1(end);
model.uInit_egr=simu_egr(end);
model.uInit_vgt=simu_vgt(end);

%% Second simulation

n_e = 1500; 
u_delta = 110;
u_egr = 80;        
u_vgt = 25;
U = [u_delta u_egr u_vgt];

% Eliminate states (for now)
x_init = [simp_im(end); simp_em(end); simX_Oim(end); simX_Oem(end); simn_t(end)*pi/30];

options = odeset('RelTol',1e-10, 'AbsTol', 1e-8);
[t_sim, X_sim] = ode15s(@(t,X)diesel_engine(X, U, n_e, model), [0 8], x_init, options); 

%% Compare with steps from model
% Requires data from simEngine in runSimEngine.m 

figure(1)
clf
subplot(511)
plot(t_sim, X_sim(:,1))
hold on
plot(simEngine.time, simEngine.p_im, 'r--')
title('p_{im}')

legend(["MATLAB model", "Simulink model"], 'Location', 'northeast')

subplot(512)
plot(t_sim, X_sim(:,2))
hold on
plot(simEngine.time, simEngine.p_em, 'r--')
title('p_{em}')

subplot(513)
plot(t_sim, X_sim(:,3))
hold on
plot(simEngine.time, simEngine.X_Oim, 'r--')
title('X_{Oim}')

subplot(514)
plot(t_sim, X_sim(:,4))
hold on
plot(simEngine.time, simEngine.X_Oem, 'r--')
title('X_{Oem}')

subplot(515)
plot(t_sim, X_sim(:,5))
hold on
plot(simEngine.time, simEngine.n_t*pi/30, 'r--')
title('\omega_t')

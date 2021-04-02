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

% Eliminate states (for later)
x_init = [simp_im(end); simp_em(end); simX_Oim(end); simX_Oem(end); simn_t(end)*pi/30];

%% Simulate step using Simulink model 

simU.u_vgt=[[0 0]' [30 25]'];

model.x_r_Init=simx_r(end);
model.T_1_Init=simT_1(end);
model.uInit_egr=simu_egr(end);
model.uInit_vgt=simu_vgt(end);
opt=simset('InitialState',xFinal);

%simulate the step
sim('TCDI_EGR_VGT',8,opt)

% Collect variables in simEngine
%temperature [K]
simEngine.T_c=simT_c;
simEngine.T_e=simT_e;
simEngine.T_em=simT_em;

%pressure [Pa]
simEngine.p_im=simp_im;
simEngine.p_em=simp_em;

%oxygen mass fraction [-]
simEngine.X_Oim=simX_Oim;
simEngine.X_Oem=simX_Oem;

%massflow [kg/s]
simEngine.W_c=simW_c;
simEngine.W_t=simW_t;
simEngine.W_ei=simW_ei;
simEngine.W_f=simW_f;
simEngine.W_egr=simW_egr;

%efficiency [-]
simEngine.eta_tm=simeta_tm;
simEngine.eta_c=simeta_c;
simEngine.BSR=simBSR;

%Power [W]
simEngine.P_c=simP_c;
simEngine.P_tm=simP_tm;

%torque [Nm]
simEngine.M_e=simM_e;

%speed [rpm]
simEngine.n_t=simn_t;
simEngine.n_e=simn_e;

%control signals
simEngine.u_delta=simu_delta;
simEngine.u_egr=simu_egr;
simEngine.u_vgt=simu_vgt;

%output
simEngine.lambda=simlambda;
simEngine.x_egr=simx_egr;

%time [s]
simEngine.time=simTime;

%% MATLAB model simulation of same step

n_e = 1500; 
u_delta = 110;
u_egr = 80;        
u_vgt = 25;
U = [u_delta u_egr u_vgt];

options = odeset('RelTol',1e-10, 'AbsTol', 1e-8);
[t_sim, X_sim] = ode15s(@(t,X)diesel_engine(X, U, n_e, model), [0 8], x_init, options); 

%% Compare with steps from model
% Requires data from simEngine in runSimEngine.m 

h = figure(1);
clf
subplot(511)
plot(simEngine.time, simEngine.p_im)
hold on
plot(t_sim, X_sim(:,1), 'r--')
ylabel('p_{im} [Pa]')

subplot(512)
plot(simEngine.time, simEngine.p_em)
hold on
plot(t_sim, X_sim(:,2), 'r--')
ylabel('p_{em} [Pa]')

subplot(513)
plot(simEngine.time, simEngine.X_Oim)
hold on
plot(t_sim, X_sim(:,3), 'r--')
ylabel('X_{Oim} [-]')

subplot(514)
plot(simEngine.time, simEngine.X_Oem)
hold on
plot(t_sim, X_sim(:,4), 'r--')
ylabel('X_{Oem} [-]')

subplot(515)
plot(simEngine.time, simEngine.n_t*pi/30)
hold on
plot(t_sim, X_sim(:,5), 'r--')
ylabel('\omega_t [rad/s]')
xlabel('Time [s]')

legend(["Simulink model", "MATLAB model"], 'Location', 'southeast')

%% Export as PDF
set(h,'Units','Inches');
pos = get(h,'Position');
set(h,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
print(h,'Figures/compMatlabModel','-dpdf','-r0')

%% Plot error

h2 = figure(2);
hold on
all_x = unique([t_sim; simEngine.time]);

refined_A = interp1(t_sim, X_sim(:,1), all_x);
refined_B = interp1(simEngine.time, simEngine.p_im, all_x);
plot(all_x, abs(refined_B - refined_A)/mean(simEngine.p_im))

refined_A = interp1(t_sim, X_sim(:,2), all_x);
refined_B = interp1(simEngine.time, simEngine.p_em, all_x);
plot(all_x, abs(refined_B - refined_A)/mean(simEngine.p_em))

refined_A = interp1(t_sim, X_sim(:,3), all_x);
refined_B = interp1(simEngine.time, simEngine.X_Oim, all_x);
plot(all_x, abs(refined_B - refined_A)/mean(simEngine.X_Oim))

refined_A = interp1(t_sim, X_sim(:,4), all_x);
refined_B = interp1(simEngine.time, simEngine.X_Oem, all_x);
plot(all_x, abs(refined_B - refined_A)/mean(simEngine.X_Oem))

refined_A = interp1(t_sim, X_sim(:,5), all_x);
refined_B = interp1(simEngine.time, simEngine.n_t*pi/30, all_x);
plot(all_x, abs(refined_B - refined_A)/mean(simEngine.n_t*pi/30))
ylabel('Mean normalized absolute error')
xlabel('Time [s]')

legend(["p_{im}", "p_{em}", "X_{Oim}", "X_{Oem}", "\omega_t"], 'Location', 'northeast')

%% Export as PDF
% set(h2,'Units','Inches');
% pos = get(h2,'Position');
% set(h2,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
% print(h2,'Figures/errorsMatlabModel','-dpdf','-r0')

%%
M_e_sim = zeros(length(t_sim), 1);
lambda_O_sim = zeros(length(t_sim), 1);

for i = 1:length(t_sim)
    [~, y, ~] = diesel_engine(X_sim(i,:), U, n_e, model);
    M_e_sim(i) = y.M_e;
    lambda_O_sim(i) = y.lambda_O;
end
% 
% h3 = figure;
% yyaxis left
% hold on
% plot(tout, simM_e)
% plot(t_sim, M_e_sim, 'r--')
% ylabel('M_e [Nm]')
% 
% yyaxis right
% hold on
% plot(tout, simlambda)
% plot(t_sim, lambda_O_sim, 'r--')
% ylabel('\lambda_O [-]')

h3 = figure;
subplot(2,1,1)
hold on
plot(tout, simM_e)
plot(t_sim, M_e_sim, 'r--')
ylabel('M_e [Nm]')

subplot(2,1,2)
hold on
plot(tout, simlambda)
plot(t_sim, lambda_O_sim, 'r--')
ylabel('\lambda_O [-]')
xlabel('Time [s]')
legend(["Simulink model", "MATLAB model"], 'Location', 'northeast')

%% Export as PDF
% set(h3,'Units','Inches');
% pos = get(h3,'Position');
% set(h3,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
% print(h3,'Figures/morecompMatlabModel','-dpdf','-r0')
tic
clc
clear all
addpath(genpath('Models\'))
%addpath Parameters\
run EATS_param

load WHTC.mat
dc_whtc = dc;
load dc.mat
orig_state = warning;
warning('off','all')
load ('ld2_NOx_0_1800_K2d3.mat','opt_signals')
warning(orig_state);

T_em = opt_signals.T_em.val;
%%
% T_temp = opt_signals.T_em.val;
% t = linspace(1,1800,10000);
% T_vec = interp1(opt_signals.t_vec.val,T_temp,t,'linear');
% T_em = ones(size(t));
% T_em(1) = 298;
% for i = 1:length(T_vec)-1
%     T_em(i+1) = T_em(i) + 0.005*(T_vec(i)-T_em(i));
% end
% t2 = linspace(1, 1800, length(opt_signals.T_em.val));
% T_em = interp1(t,T_em,t2);
% 
% hold on
% plot(opt_signals.t_vec.val,T_temp,'color',[0,0.4470,0.7410,0.4],'LineWidth',0.2)
% plot(t2,T_em)
% title('Result of filtering engine out temperature')
% xlabel('time')
% ylabel('Temperature [°C]')
% legend('Measured','Filtered')
%%
% Exogenous inputs

% Original
% T_amb = 300;
% W = @(t) interp1(dc.t,dc.W_ats,t+dc.t(1));
% T_bDOC = @(t) interp1(dc.t,dc.T_bDOC,t+dc.t(1));

% Scania WHTC
% T_amb = 300;
% W = @(t) interp1(dc_whtc.t,dc_whtc.W_a,t+dc_whtc.t(1));
% T_bDOC = @(t) interp1(dc_whtc.t,dc_whtc.T_bDOC+273,t+dc_whtc.t(1));
% p = 1e5; %@(t) interp1(dc_whtc.t, dc_whtc.p_em, t+dc_whtc.t);

% Simulated WHTC
T_amb = 300;
W = @(t) interp1(opt_signals.t_vec.val,opt_signals.W_tw.val,t+opt_signals.t_vec.val(1));
T_bDOC = @(t) interp1(opt_signals.t_vec.val,T_em+273,t+opt_signals.t_vec.val(1));
p = 1e5; %@(t) interp1(dc_whtc.t, dc_whtc.p_em, t+dc_whtc.t);

V = @(t) [W(t); 
    p;%(t); 
    T_bDOC(t); 
    T_amb];

% Controlsignals
U = [];

% Initial conditions
% T_0 = 300; % Useless?
% T_DOC_0 = ones(EATS.DOC.n_seg,1)*(273+277);
% T_DPF_0 = ones(EATS.DPF.n_seg,1)*(273+269);
% T_SCR_0 = ones(EATS.SCR.n_seg,1)*(273+260);
%   T_silencer_0 = [300+100 300+60]';
X_0 = [EATS.T_DOC_0; EATS.T_DPF_0; EATS.T_SCR_0; EATS.T_silencer_0];

% Simulate
simulation_model = @(t,X) EATS_model_test(X,V(t),U,EATS,[]);
t_span = [0 1800];%[0 500*2*2*2.5];

[t,X] = ode15s(simulation_model,t_span,X_0,odeset('RelTol',1e-6));

% Extract signals
signals = [];
for n = 1 : length(t)
    [~, signals] = EATS_model_test(X(n,:)',V(n),U,EATS,signals);
end
toc
%% Plot whtc
figure(4);
clf;
hold on
ax = subplot(221); hold on
grid on
plot(opt_signals.t_vec.val,opt_signals.T_atw.val,'LineWidth',2) % Unfiltered
%plot(t2,T_em)                                    % Filtered
plot(dc_whtc.t/10, dc_whtc.T_bDOC,'LineWidth',2)
title('T_bDOC')
ylabel('Temperature [°C]')
xlabel('Time [s]')
legend('Modeled','Measured')

ax(2) = subplot(222); hold on
grid on
plot(t,signals.T_aDOC-273,'LineWidth',2)
plot(dc_whtc.t/10,dc_whtc.T_aDOC,'LineWidth',2)
%plot(t,signals.T_inside,'--')
%plot(t,signals.T_shell,'--')
title('T_aDOC')
ylabel('Temperature [°C]')
xlabel('Time [s]')
legend('Modeled', 'Measured','Location','SouthEast')

ax(3) = subplot(223); hold on
grid on
plot(t,signals.T_aDPF-273,'LineWidth',2)
plot(dc_whtc.t/10,dc_whtc.T_aDPF,'LineWidth',2)
title('T_aDPF')
ylabel('Temperature [°C]')
xlabel('Time [s]')
legend('Modeled', 'Measured','Location','SouthEast')

ax(4) = subplot(224); hold on
grid on
plot(t,signals.T_SCR-273,'LineWidth',2)
title('T_aSCR')
ylabel('Temperature [°C]')
%plot(dc.t,dc.T_aSCR-273) % No WHTC data available
legend('Modeled','Location','SouthEast')
linkaxes(ax,'x');
xlabel('Time [s]')
xlim(t_span)
%% Plot
% figure(1);
% clf;
% hold on
% ax = subplot(411)
% plot(dc.t,dc.T_bDOC-273)
% 
% ax(2) = subplot(412); hold on
% plot(t,signals.T_aDOC - 273)
% plot(dc.t,dc.T_aDOC-273)
% plot(t,signals.T_inside-273,'--')
% plot(t,signals.T_shell-273,'--')
% 
% ax(3) = subplot(413); hold on
% plot(t,signals.T_aDPF - 273)
% plot(dc.t,dc.T_aDPF-273)
% 
% ax(4) = subplot(414); hold on
% plot(t,signals.T_aSCR - 273)
% plot(dc.t,dc.T_aSCR-273)
% 
% 
% linkaxes(ax,'x');
% xlim(t_span)



function [deNOx_Cu1, deNOx_Fe1, deNOx_Cu2, deNOx_Fe2, signals1, signals2, t1, t2] = test_fun(data1, data2)
tic
clc
addpath(genpath('Models\'))
%addpath Parameters\
run EATS_param
close all
orig_state = warning;
warning('off','all')
warning(orig_state);


%% Exogenous inputs

% Function input arguments-------------------------------------------------
data = data1;
T_amb = 300;
W1 = @(t1) interp1(data.t_vec.val, data.W_tw.val, t1+data.t_vec.val(1));
T_bDOC1 = @(t1) interp1(data.t_vec.val, data.T_atw.val+273,t1+data.t_vec.val(1));
p = 1e5;
%
data = data2;
T_amb = 300;
W2 = @(t2) interp1(data.t_vec.val, data.W_tw.val, t2+data.t_vec.val(1));
T_bDOC2 = @(t2) interp1(data.t_vec.val, data.T_atw.val+273,t2+data.t_vec.val(1));
p = 1e5;

% -------------------------------------------------------------------------
% Input vectors
V1 = @(t1) [W1(t1); 
    p;%(t); 
    T_bDOC1(t1); 
    T_amb];
V2 = @(t2) [W2(t2); 
    p;%(t); 
    T_bDOC2(t2); 
    T_amb];

% Control signals
U = [];
X_0 = [EATS.T_DOC_0; EATS.T_DPF_0; EATS.T_SCR_0; EATS.T_silencer_0];

% Simulate
simulation_model1 = @(t1,X) EATS_model_test(X,V1(t1),U,EATS,[]);
simulation_model2 = @(t2,X) EATS_model_test(X,V2(t2),U,EATS,[]);
t_span = [0 1800];%[0 500*2*2*2.5];

[t1,X1] = ode15s(simulation_model1,t_span,X_0,odeset('RelTol',1e-6));
[t2,X2] = ode15s(simulation_model2,t_span,X_0,odeset('RelTol',1e-6));

% Extract signals
signals1 = [];
for n = 1 : length(t1)
    [~, signals1] = EATS_model_test(X1(n,:)',V1(n),U,EATS,signals1);
end
signals2 = [];
for n = 1 : length(t2)
    [~, signals2] = EATS_model_test(X2(n,:)',V2(n),U,EATS,signals2);
end

% deNOx performance
[deNOx_Cu1, deNOx_Fe1] = deNOx_test_fun(signals1.T_SCR, data1);
[deNOx_Cu2, deNOx_Fe2] = deNOx_test_fun(signals2.T_SCR, data2);
end

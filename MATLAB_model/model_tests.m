%% MATLAB model testing
% run first section of run.m

addpath('MATLAB_model')

%% compressor.m OK

boolT_c = [];
boolP_c = [];
boolW_c = [];

for i = 1:length(simp_im)

    p_im = simp_im(i);
    w_t = simn_t(i)*pi/30;
    [W_c, T_c, P_c] = compressor(model.T_amb, model.p_amb, p_im, w_t, model);

    boolT_c = [boolT_c; ismembertol(simT_c(i), T_c)];
    boolP_c = [boolP_c; ismembertol(simP_c(i), P_c)];
    boolW_c = [boolW_c; ismembertol(simW_c(i), W_c)];

end

%% turbine.m OK

u_vgt = 75;

boolW_t = [];
boolP_t_eta_m = [];

for i = 1:length(simW_t)

    T_em = simT_em(i);
    p_em = simp_em(i);
    w_t = simn_t(i)*pi/30;
    [W_t, P_t_eta_m] = turbine(T_em, p_em, model.p_amb, w_t, u_vgt, model);

    boolP_t_eta_m = [boolP_t_eta_m; ismembertol(simP_tm(i), P_t_eta_m)];
    boolW_t = [boolW_t; ismembertol(simW_t(i), W_t)];
    
end

%% EGR_system.m OK

u_egr = 80;  

boolW_egr = [];

for i = 1:length(simW_egr)

    p_im = simp_im(i);
    T_em = simT_em(i);
    p_em = simp_em(i);
    W_egr = EGR_system(p_im, T_em, p_em, u_egr, model);

    boolW_egr = [boolW_egr; ismembertol(simW_egr(i), W_egr)];
    
end

%% cylinder.m

n_e = 1500; 
u_delta = 110;

boolW_ei = [];
boolW_eo = [];
T_ems = []; % Appoximated therefore better to investigate separately
% boolX_Oe = []; % not logged

for i = 1:length(simW_ei)

    p_im = simp_im(i);
    p_em = simp_em(i);
    X_Oim = simX_Oim(i);
    [W_ei, W_eo, T_em, X_Oe] = ...
    cylinder(p_im, p_em, X_Oim, n_e, u_delta, model.T_im, model);

    boolW_ei = [boolW_ei; ismembertol(simW_ei(i), W_ei)];
    boolW_eo = [boolW_eo; ismembertol(simW_eo(i), W_eo)];
    T_ems = [T_ems; T_em];
end

figure
plot(tout, simT_em)
hold on
plot(tout, T_ems, 'r--')
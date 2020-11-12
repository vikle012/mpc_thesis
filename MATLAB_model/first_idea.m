% Model

clear

load parameterData

control.u_egract=1;
% 1: with EGR-actuator dynamics
% 0: without EGR-actuator dynamics

control.u_vgtact=1;
% 1: with VGT-actuator dynamics
% 0: without VGT-actuator dynamics

% Inputs
n_e=[0 1500]; 
u_delta=[0 110];
u_egr=[0 80];        
u_vgt=[0 75];

% Time loop
t0 = clock;
simulation_time = 10;

while etime(clock, t0) < simulation_time
    pause(0.05)
    disp(clock)
end

% Logged Variables:
% T_c
% T_e
% T_em
% T_1
% x_r
% p_im      OK
% p_em      OK
% BSR
% W_egr
% W_c
% W_t
% W_ei
% W_eo
% W_f
% eta_tm
% eta_c
% P_c
% P_tm
% M_e
% n_t
% n_e
% lambda
% X_Oim     OK
% X_Oem     OK
% u_delta
% u_egr
% u_vgt
% u_egract
% u_vgract
% x_egr     OK
% simTime

% Intake Manifold
function [p_im, X_Oim] = intake_mainfold(W_c, W_egr, W_ei, T_im, X_Oem)

    p_im_dot = model.R_a*T_im*(W_c + W_egr - W_ei)/model.V_im;
    % p_im = 
    
    % Subsystem X_Oim --> TODO: X_Oim fås hur? Tidigare värden
    X_Oim_dot = model.R_a*T_im*((X_Oem - X_Oim)*W_egr + (X_Oc - X_Oim)*W_c)/(p_im*model.V_im);
    % X_Oim = 
    
    % Logged variables
    variables.x_egr = min(W_egr/(W_c + W_egr), 0.9);
    variables.p_im = p_im;
    variables.X_Oim = X_Oim;
end

% Exhaust mainfold
function [p_em, X_Oem] = intake_mainfold(W_eo, W_t, W_egr, T_em, X_Oe)

    p_em_dot = model.R_e*T_em*(W_eo - W_t - W_egr)/model.V_em;
    
    % Integrator? Numerical? Använda persistent?
    % p_em = 
    
    % Subsystem X_Oim --> TODO: X_Oem fås hur?
    X_Oim_dot = model.R_e*T_em*(X_Oe - X_Oem)*W_eo/(p_im*model.V_im);
    % X_Oem = 
    
    % Logged variables
    variables.p_em = p_em;
    variables.X_Oem = X_Oem;
end

% Cylinder
function [W_ei, W_eo, T_em, X_Oe] = EGR_system(p_im, p_em, X_Oim, n_e, u_delta, T_im)
    W_ei = ;
    W_eo = ; 
    T_em = ;
    X_Oe = ;
end

% Turbo
function [Wc, W_t] = turbo(p_im, p_em, T_em, u_vgt)

    % --- Turbine ---
    % in: T_em, p_em, p_es, omega_t, u_vgt
    % out: W_t, P_t_eta_m
    p_es = model.p_amb;
    PI_t = min(p_es/p_em, 0.9999);
    
    BSR = model.R_t*omega_t/(2*model.c_pe*T_em*(1 - PI_t^(1 - 1/model.gamma.e)));
    BSR = min(2*model.BSR_opt, max(0, BSR));
    
    % TODO: något fel i Simulink här?
    c_m = model.c_mVec(1)*(max(0, omega_t - model.c_mVec(2)))^model.c_mVec(3);
    
    eta_tm = model.eta_tmmax - c_m*(BSR - model.BSR_opt)^2;
    eta_tm = min(1, max(0.2, eta_tm));
    
    
    u_vgt = min(100, max(20, u_vgt)); % Saturation
    
    if control.u_vgtact
        % VGT act
        
        % TODO: Transport delay
        % ...
        
        u_vgtact = 
    else
        u_vgtact = u_vgt; 
    end
    
    f_vgt = max(0, model.c_f2 + model.c_f1*sqrt(max(0, 1 - ((u_vgtact - model.c_vgt2)/model.c_vgt1)^2)));
  
    W_t = model.A_vgtmax*p_em*sqrt(1 - PI_t^model.K_t)*f_vgt/sqrt(T_em*model.R_e);
    
    P_t_eta_m = model.c_pe*W_t*T_em*(1 - PI_t^(1 - 1/model.gamma_e))*eta_tm;
    
    % --- Compressor ---
    
    % --- Turbo inertia ---
    omega_t_dot = (P_t_eta_m - P_c)/(model.J_t*omega_t);
    omega_t = ;
    
    % Logged variables
    variables.BSR = [variables.BSR BSR];    
    variables.T_c = [variables.T_c T_c]; % TODO kolla upp med ovan
end

% EGR-system
function W_egr = EGR_system(p_im, T_em, p_em, u_egr)
    
    u_egr = min(100, max(0, u_egr)); % Saturation

    if control.uegract
        % EGR act
        
        % TODO: Transport delay
        
        %u_egr1_dot = .../model.tau_egr1;
        %u_egr2_dot = .../model.tau_egr2;
        
        % Integrate
        %u_egr1 = ;
        %u_egr2 = ;
        
        u_egract = model.K_egr*u_egr1 - (model.K_egr - 1)*u_egr2;
    else
        u_egract = u_egr;
    end
    
    if u_egract > -(model.c_egr(2)/(2*model.c_egr(1)))
        f_egr = model.c_egr(3) - model.c_egr(2)^2/(4*model.c_egr(1));
    else
        f_egr = model.c_egr(1)*u_egract^2 + model.c_egr(2)*u_egract + ...
                model.c_egr(3);
    end
        
    A_egr = model.A_egrmax*f_egr;

    PI_egr = min(1, max(model.PI_egropt, p_im/p_em)); % Saturation
    PSI_egr = 1 - ((1 - PI_egr)/(1 - model.PI_egropt) - 1)^2;   
    
    W_egr = A_egr*p_em*PSI_egr/sqrt(T_em*model.R_e);
    
    % Logged variables
    variables.u_egr = [variables.u_egr u_egr];
    variables.u_egract = [variables.u_egract u_egract];
    variables.W_egr = [variables.W_egr W_egr];
end
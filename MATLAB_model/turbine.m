function [W_t, P_t_eta_m, eta_tm, BSR, PI_t] = turbine(T_em, p_em, p_es, w_t, u_vgt, param)
    
    %% Model Parameters
    R_t         = param.R_t;
    c_pe        = param.c_pe;
    gamma_e     = param.gamma_e;
    BSR_opt     = param.BSR_opt;
    c_m1        = param.c_mVec(1);
    c_m2        = param.c_mVec(2);
    c_m3        = param.c_mVec(3);
    eta_tmmax	= param.eta_tmmax;
    c_f1        = param.c_f1;
    c_f2        = param.c_f2;
    c_vgt1      = param.c_vgt1;
    c_vgt2      = param.c_vgt2;
    A_vgtmax    = param.A_vgtmax; 
    K_t         = param.K_t;
    R_e         = param.R_e;
    
    %% Calculations
    
    % -- In PI_t --
    PI_t = p_es/p_em; % min(p_es/p_em, 0.9999); % "Saturation"
    
    % -- In eta_tm --
    BSR = R_t*w_t/sqrt(2*c_pe*T_em*(1 - PI_t^(1 - 1/gamma_e)));
    % BSR = min(2*BSR_opt, max(0, BSR)); % Saturation  
    c_m = c_m1*(max(0, w_t - c_m2))^c_m3;
    eta_tm = eta_tmmax - c_m*(BSR - BSR_opt)^2;
    % eta_tm = min(1, max(0.2, eta_tm)); % Saturation
    eta_tm_DeltaT_t = T_em*(1 - PI_t^(1 - 1/gamma_e))*eta_tm;

    % u_vgt = min(100, max(20, u_vgt)); % Saturation   
    
    % No actuator dynamics
    u_vgtact = u_vgt;
    
    % -- In Subsystem W_t --
    f_vgt = max(0, c_f2 + c_f1*sqrt(max(0, 1 - ((u_vgtact - c_vgt2)/c_vgt1)^2)));
    W_t = A_vgtmax*p_em*sqrt(1 - PI_t^K_t)*f_vgt/sqrt(T_em*R_e);
    
    P_t_eta_m = W_t*c_pe*eta_tm_DeltaT_t;
end
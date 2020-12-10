function [W_ei, W_eo, T_em, X_Oe, lambda_O, lambda_air, M_e, x_r, T_1] = ...
    cylinder(p_im, p_em, X_Oim, n_e, u_delta, T_im, param)
    
    %% Model Parameters
    V_d         = param.V_d;
    R_a         = param.R_a;
    c_volVec    = param.c_volVec;
    n_cyl       = param.n_cyl;
    AFs         = param.AFs;
    X_Oc        = param.X_Oc;
    q_HV        = param.q_HV;
    eta_igch    = param.eta_igch;
    r_c         = param.r_c;
    gamma_c     = param.gamma_c;
    c_fricVec   = param.c_fricVec;
    T_w         = param.T_w;
    h_tot       = param.h_tot;
    d_pipe      = param.d_pipe;
    l_pipe      = param.l_pipe;
    n_pipe      = param.n_pipe;
    c_pe        = param.c_pe;
    x_cv        = param.x_cv;
    c_va        = param.c_va;
    gamma_a     = param.gamma_a;
    eta_sc      = param.eta_sc;
    c_pa        = param.c_pa;
    
    %% Calculations
    
    % n_e = min(2000, max(500, n_e)); % Saturation
    % u_delta = min(250, max(1, u_delta)); % Saturation
    
    % -- In Cylinder flow --
    eta_vol = c_volVec(1)*sqrt(p_im) + c_volVec(2)*sqrt(n_e) + c_volVec(3);
    W_ei = eta_vol*p_im*n_e*V_d/(120*R_a*T_im);
    
    % -- In W_f --
    W_f = 1e-6*u_delta*n_e*n_cyl/120; % Fuel mass flow

    % -- In lambda --
    lambda_O = W_ei*X_Oim/(W_f*AFs*X_Oc);
    W_eo = W_f + W_ei;
    X_Oe = (W_ei*X_Oim - W_f*AFs*X_Oc)/W_eo; % "Saturation"
    
    % for constraints in OCP
    lambda_air =  W_ei/(W_f*AFs);
    
    % -- In cylinder torque --
    lambda = min(1, lambda_O); % Scaling factor
    W_ig = eta_igch*lambda*(1 - 1/(r_c^(gamma_c - 1)))*u_delta*q_HV*1e-6*n_cyl;
    W_p = V_d*(p_em - p_im);
    n_eratio = n_e/1000;
    W_fric = V_d*1e5*(c_fricVec(1)*n_eratio^2 + c_fricVec(2)*n_eratio + c_fricVec(3));
    M_e = (W_ig - W_p - W_fric)/(4*pi);
    
    % -- In Cylinder temp (one iter with delay) --
    % Approximated using equation (24)   
    PI_e = p_em/p_im;
    
    x_r = param.x_r_Init;
    T_1 = param.T_1_Init;
    for i = 1:5
        q_in = W_f*q_HV*(1 - x_r)/(W_f + W_ei);
        x_p = 1 + q_in*x_cv/(c_va*T_1*r_c^(gamma_a - 1));
        x_v = 1 + q_in*(1 - x_cv)/(c_pa*(q_in*x_cv/c_va + T_1*r_c^(gamma_a - 1)));
        x_r = PI_e^(1/gamma_a)*x_p^(-1/gamma_a)/(r_c*x_v);
        x_r = min(1, max(0, x_r)); % Saturation
        T_e = eta_sc*PI_e^(1 - 1/gamma_a)*r_c^(1 - gamma_a)*x_p^(1/gamma_a - 1)*...
        (q_in*((1 - x_cv)/c_pa + x_cv/c_va) + T_1*r_c^(gamma_a - 1));
        T_e = min(1500, T_e); % Saturation
        T_1 = x_r*T_e + (1 - x_r)*T_im;
        T_1 = max(273, T_1); % Saturation
    end
    
    % -- In Exhaust temp drop --
    T_em = T_w + (T_e - T_w)*exp(-1*h_tot*pi*d_pipe*l_pipe*n_pipe/(W_eo*c_pe));  
    
end
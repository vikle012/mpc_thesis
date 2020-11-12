function [W_c, T_c, P_c] = compressor(T_bc, p_bc, p_c, w_t, param)
    
    %% Model parameters
    R_a         = param.R_a;
    R_c         = param.R_c;
    c_pa        = param.c_pa;
    gamma_a     = param.gamma_a;
    c_wpsiVec   = param.c_wpsiVec;
    c_wphiVec   = param.c_wphiVec;
    c_phi2      = param.c_phi2;
    c_psi2      = param.c_psi2;
    W_copt      = param.w_copt;
    pi_copt     = param.pi_copt;
    c_pi        = param.c_pi;
    Q_c         = param.Q_c;
    eta_cmax    = param.eta_cmax;

    %% Calculations
    
    % -- In Sub PI_c --
    PI_c = max(1, p_c/p_bc);
      
    % -- In Sub W_c --
    PSI_c = 2*c_pa*T_bc*(PI_c^(1 - 1/gamma_a) - 1)/(R_c^2*w_t^2);
    c_psi1 = max(0, c_wpsiVec(1)*w_t^2 + c_wpsiVec(2)*w_t + c_wpsiVec(3));
    c_phi1 = max(0.1, c_wphiVec(1)*w_t^2 + c_wphiVec(2)*w_t + c_wphiVec(3));
    fcn = 1 - c_psi1*(PSI_c - c_psi2)^2;
    fcn = max(0, fcn);
    fcn1 = c_phi2 + sqrt(fcn/c_phi1);
    PHI_c = max(0, fcn1); 
    W_c = p_bc*pi*R_c^3*w_t*PHI_c/(R_a*T_bc);
    W_c = max(1e-4, W_c); % Saturation    
    
    % -- In DeltaT_c --
    pi_c = (PI_c - 1)^c_pi;
    chi = [W_c - W_copt; pi_c - pi_copt]; 
    eta_c = max(0.2, eta_cmax - chi'*Q_c*chi);
    DeltaT_c = T_bc*(PI_c^(1 - 1/gamma_a) - 1)/eta_c;
    T_c = DeltaT_c + T_bc;
    
    P_c = W_c*c_pa*DeltaT_c;
end


function W_egr = EGR_system(p_im, T_em, p_em, u_egr, param)

    %% Model Parameters
    c_egr1      = param.c_egr(1);
    c_egr2      = param.c_egr(2);
    c_egr3      = param.c_egr(3);
    A_egrmax    = param.A_egrmax;
    PI_egropt   = param.PI_egropt;
    R_e         = param.R_e;

    %% Calculations
    
    u_egr = min(100, max(0, u_egr)); % Saturation

    % No actuator dynamics
    u_egract = u_egr;
    
    % -- In f_egr --
    f_egr = max(0, c_egr1*min(u_egract, -c_egr2/(2*c_egr1))^2 + ...
                   c_egr2*min(u_egract, -c_egr2/(2*c_egr1)) + c_egr3);
        
    A_egr = A_egrmax*f_egr;
    
    % -- In PSI_egr --
    PI_egr = min(1, max(PI_egropt, p_im/p_em)); % Saturation
    PSI_egr = 1 - ((1 - PI_egr)/(1 - PI_egropt) - 1)^2;   
    
    W_egr = A_egr*p_em*PSI_egr/sqrt(T_em*R_e);
end
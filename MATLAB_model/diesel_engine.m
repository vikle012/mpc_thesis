function [dX, signals, constr] = diesel_engine(X, U, n_e, param)
%% Model ------------------------------------------------------------------
    % States
    p_im        = X(1); % Intake manifold pressure                  [Pa]
    p_em        = X(2); % Exhaust manifold pressure                 [Pa]
    X_Oim       = X(3); % Intake manifold oxygen mass fraction      [-]
    X_Oem       = X(4); % Exhaust manifold oxygen mass fraction     [-]
    w_t         = X(5); % Turbocharger rotational speed             [rad/s]
    %u_egract1   = X(6); % EGR actuator position 1                  [%]
    %u_egract2   = X(7); % EGR actuator position 2                  [%]
    %u_vgtact    = X(8); % VGT actuator position                    [%]

    % Control signals
    u_delta = U(1); % Fuel injection        [mg/cycle]
    u_egr   = U(2); % EGR control signal    [%]
    u_vgt   = U(3); % VGT control signal    [%]
    
    % --- Cylinder ---
    [W_ei, W_eo, T_em, X_Oe, lambda_O, lambda_air, M_e] = ...
        cylinder(p_im, p_em, X_Oim, n_e, u_delta, param.T_im, param);

    % --- Turbo ---
    % Compressor
    [W_c, T_c, P_c, PI_c, eta_c] = ...
        compressor(param.T_amb, param.p_amb, p_im, w_t, param);

    % Turbine
    [W_t, P_t_eta_m, eta_tm, BSR, PI_t] = ...
        turbine(T_em, p_em, param.p_amb, w_t, u_vgt, param);

    % --- EGR-system ---
    [W_egr, PI_egr] = ...
        EGR_system(p_im, T_em, p_em, u_egr, param);

    % Dynamics
    d_p_im = param.R_a*param.T_im*(W_c + W_egr - W_ei)/param.V_im;    
    d_p_em = param.R_e*T_em*(W_eo - W_t - W_egr)/param.V_em;    
    d_X_Oim = param.R_a*param.T_im*((X_Oem - X_Oim)*W_egr + (param.X_Oc - X_Oim)*W_c)/(p_im*param.V_im);   
    d_X_Oem = param.R_e*T_em*(X_Oe - X_Oem)*W_eo/(p_em*param.V_em);
    d_w_t = (P_t_eta_m - P_c)/(param.J_t*w_t);
    
    dX = [d_p_im; d_p_em; d_X_Oim; d_X_Oem; d_w_t];

%% Saturation constraints
    
    % --- From turbine.m ---
    % PI_t <= 0.9999            "Saturation"
    % 0 <= BSR <= 2*BSR_opt      Saturation
    % 0.2 <= eta_tm <= 1         Saturation
    constr_t = [PI_t; BSR; eta_tm];
    constr_t_lbg = [0; 0; 0.2];
    constr_t_ubg = [0.9999; 2*param.BSR_opt; 1];

    % --- From EGR_system.m ---
    % PI_egropt <= PI_egr <= 1   Saturation
    constr_EGR = PI_egr;
    constr_EGR_lbg = param.PI_egropt;
    constr_EGR_ubg = 1;

    % --- From cylinder.m ---
    % 1.2 < lambda_air      Discussed value
    % 0 <= X_Oe             "Saturation"
    constr_cyl = [lambda_air; X_Oe];
    constr_cyl_lbg = [1.2; 0];
    constr_cyl_ubg = [inf; inf;];

    % --- From compressor.m ---
    % 1 <= PI_c                 "Saturation"
    % 1e-4 <= W_c <= inf         Saturation
    % 0.2 <= eta_c              "Saturation"
    constr_c = [PI_c; W_c; eta_c];
    constr_c_lbg = [1; 1e-4; 0.2];
    constr_c_ubg = [inf; inf; 1];

    constr.constraints = [constr_t; constr_EGR; constr_cyl; constr_c];
    constr.lbg = [constr_t_lbg; constr_EGR_lbg; constr_cyl_lbg; constr_c_lbg];
    constr.ubg = [constr_t_ubg; constr_EGR_ubg; constr_cyl_ubg; constr_c_ubg];

%% Signals

    signals.X_Oe = X_Oe;
    signals.W_eo = W_eo;
    signals.W_ei = W_ei;
    
    signals.lambda_air = lambda_air;
    signals.M_e = M_e;
    signals.W_egr = W_egr;
    signals.W_c = W_c;
    signals.eta_tm = eta_tm;
    signals.BSR = BSR;
    signals.PI_t = PI_t;
    signals.PI_egr = PI_egr;
    signals.eta_c = eta_c;
    
    signals.x_egr = W_egr/(W_c + W_egr);
    signals.lambda_O = lambda_O;
end


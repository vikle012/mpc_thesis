function [dX, signals, param] = diesel_engine(X, U, n_e, param)
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
    [W_ei, W_eo, T_em, X_Oe, lambda_O, M_e] = ...
        cylinder(p_im, p_em, X_Oim, n_e, u_delta, param.T_im, param);

    % --- Turbo ---
    % Compressor
    [W_c, T_c, P_c] = compressor(param.T_amb, param.p_amb, p_im, w_t, param);

    % Turbine
    [W_t, P_t_eta_m] = turbine(T_em, p_em, param.p_amb, w_t, u_vgt, param);

    % --- EGR-system ---
    W_egr = EGR_system(p_im, T_em, p_em, u_egr, param);

    % Dynamics
    d_p_im = param.R_a*param.T_im*(W_c + W_egr - W_ei)/param.V_im;    
    d_p_em = param.R_e*T_em*(W_eo - W_t - W_egr)/param.V_em;    
    d_X_Oim = param.R_a*param.T_im*((X_Oem - X_Oim)*W_egr + (param.X_Oc - X_Oim)*W_c)/(p_im*param.V_im);   
    d_X_Oem = param.R_e*T_em*(X_Oe - X_Oem)*W_eo/(p_em*param.V_em);
    d_w_t = (P_t_eta_m - P_c)/(param.J_t*w_t);
    
    dX = [d_p_im; d_p_em; d_X_Oim; d_X_Oem; d_w_t];

%% Signals

    signals.X_Oe = X_Oe;
    signals.W_eo = W_eo;
    signals.W_ei = W_ei;
    signals.lambda_O = lambda_O;
    signals.M_e = M_e;
    signals.W_egr = W_egr;
    signals.W_c = W_c;
    
end


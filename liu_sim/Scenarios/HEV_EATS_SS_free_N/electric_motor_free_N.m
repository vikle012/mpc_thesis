function [dX, constraints, signals] = electric_motor_free_N(X, U, param)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Electric motor model
% [SOC] = states X(5)
% [M_em] = controls U(4)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Unit conversions etc.
SOC     = X(1,:);

M_em    = U(1,:);
N_em    = U(2,:);
w_em    = N_em.*pi./30;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Motor
P_em_mech   = w_em .* M_em;

% Efficiency approach for losses (does not include friction at 0 torque)
%lut_eta_em  = param.em.lut_eta_em;
%eta_em      = full(lut_eta_em([w_em M_em]));
%P_em_elec   = power_em(M_em, w_em, eta_em);

% P_loss approach for losses (includes friction at 0 torque)
lut_ploss = param.em.ploss_lut;
P_em_elec   = full(lut_ploss([w_em M_em])) + P_em_mech;
% PROBLEM OM P_em_mech = 0 (divide by 0)
eta_em = (P_em_mech./P_em_elec) .^ ((P_em_mech ./ sqrt(P_em_mech .^ 2)));

% Battery
U_oc        = SOCtoUoc(SOC, param.batpack);
U_b         = battery_voltage(P_em_elec, U_oc, param.batpack);
I_b         = battery_current(P_em_elec, U_b);
eta_b       = battery_efficiency(P_em_elec, battery_power(U_oc, I_b));

%% Dynamics
dX = -I_b ./ param.batpack.Q; %[dSOC/dt]
%% Constraints
constraints = [...
    M_em_max_fn(M_em, N_em); ...
    M_em_min_fn(M_em, N_em); ...
    I_b - param.batpack.Imax; ...
    param.batpack.Imin - I_b; ... 
    SOC - param.batpack.SOC_max; ...
    param.batpack.SOC_min - SOC ...
    ];

%% Signals
signals=struct(...
    ... Battery and electric motor
    'P_em_mech'  ,P_em_mech , ...
    'eta_em'     ,eta_em .* 100 , ...
    'P_em_elec'  ,P_em_elec , ...
    'U_oc'  ,U_oc , ...
    'U_b'   ,U_b  , ...
    'I_b'   ,I_b  , ...
    'M_em'  ,M_em, ...
    'eta_b' ,eta_b .* 100, ...
    'SOC'   ,SOC, ...
    'P_b'   ,U_b .* I_b ...
    );
    end


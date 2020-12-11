function param = createParam__hev_eats_lightoff(engine_map, regen_brake)
%% BUILD PARAM
% Load steady-state setpoint from the ld2 engine map
load('ld2_pwl_param.mat', 'pwl_param');
param.ld2 = pwl_param;

%% VARIABLES
load('batcell_2_3Ah.mat', 'batcell');

batcell.NoCells.series      = 197;  % Cells in series
batcell.NoCells.parallel    = 8;    % Cells in parallel
batcell.SOC         	= 0.375;    % Initially charged to 50%
batcell.SOC_max       	= 0.5;      % Max SOC [0.2 - 0.8]
batcell.SOC_min        	= 0.25;     % Min SOC [0.2 - 0.8]

% Limits in solver on engine speed [rpm]
N_min                  	= 400;
N_max                  	= 2500;

%% BATTERY

% Add all battery data to param struct
param.batpack = buildBatteryPack(batcell);

%% ELECTRIC MOTOR
load('em_155kW_3000rpm.mat', 'em');

% Create look-up table for em_eta and add it to param struct
param.em.ploss_lut = em_ploss_table(em);

%% NOx AND NO LOOK-UP TABLES

% Create look-up tables for NO and NO2/NO and add them to param struct
param.ld2.lut_NOx   = NOx_table();
param.ld2.constr.torque_max_fn = @(N,M) ...
    M - [(2.350053575605094e-05.*N.^3 - 0.037356370305584.*N.^2 + ...
    20.861397967062427.*N - 2.731879241606560e+03); ...
    2396; ...
    (6.128830375899390e-04.*N.^2 + 3.100167284419236.*N + 5.373603733118146e+03); ...
    (7.715040623355741e-04.*N.^2 - 5.018158616034680.*N + 8.440680865836488e+03) ...
    ];

%% Engine aftertreatment system
run EATS_param
param.EATS = EATS;

%% WHTC

% All WHTC data and look-up tables is contained in param.WHTC
param.WHTC = createWHTC(engine_map, regen_brake);

%% MAX AND MIN VALUES

% Max states and controls according to pwl formulation
param.ld2.constr.x_max = [pwl_param.constr.x_max(1:4); ...	ld2 states
    1e5; ...                              NOx max
    N_max; ...                          Engine speed max
    param.batpack.SOC_max; ...          SOC max
    ones(7, 1).*1500; ... % Aftertreatment states
    473.5; ...
    ones(4, 1).*1500; ...
    pwl_param.constr.x_max(5); ...      u_f max
    1; ...                              Throttle max
    1; ...                              Wastegate max (0 to disable)
    1500 ...                            M_em max (0 to disable)
    ];

% Min states and controls according to pwl formulation
param.ld2.constr.x_min = [pwl_param.constr.x_min(1:4); ... 	ld2 states
    0; ...                              NOx min
    N_min; ...                          Engine speed min
    param.batpack.SOC_min; ...          SOC min
    zeros(EATS.DOC.n_seg + EATS.DPF.n_seg + EATS.SCR.n_seg + 2, 1); ...
    pwl_param.constr.x_min(5); ...      ld2 control
    1; ...                              Throttle min  (1 to disable)
    0; ...                              Wastegate min
    -1500 ...                           M_em min (0 to disable)
    ];

% Max and min derivative of controls which are controlled
param.ld2.constr.u_max = [pwl_param.constr.u_max; ...   % ld2 max control
    30000 ...       % dM_em max
    ];    
param.ld2.constr.u_min = [pwl_param.constr.u_min; ...   % ld2 min control
    -30000 ...      % dM_em min
    ];

%% NORMALIZATION
% States and control
param.ld2.ocp.state_norm = [pwl_param.ocp.state_norm(1:4); ... % ld2 states
    1e5; ...                                % NOx norm
    N_max; ...                            	% Engine speed norm
    param.batpack.SOC_max; ...              % SOC norm
    ones(EATS.DOC.n_seg + EATS.DPF.n_seg + EATS.SCR.n_seg + 2, 1).*1500; ...
    pwl_param.ocp.state_norm(5:end); ...   	% ld2 control norm
    1500 ...                                % M_em norm
    ];

% Derivative of controls
param.ld2.ocp.control_norm = [pwl_param.ocp.control_norm; ... % ld2 control
    30000 ...       % dM_em norm
    ];

end

% -------------------------------------------------------------
% -- Main script for parallel HEV with aftertreatment system --
% -------------------------------------------------------------

clc
fprintf('Parallel HEV with aftertreatment system!\n\n');

%% Loading Battery                  -- Instant
load('batcell_2_3Ah.mat');

batcell.NoCells.series      = 197;  % Cells in series
batcell.NoCells.parallel    = 8;   % Cells in parallel
batcell.SOC                 = 0.5;  % Initially charged to 50%
batcell.SOC_max             = 0.5;  % [Desired SOC constraints -
batcell.SOC_min             = 0.25; %  between 0.2-0.8.]

batpack = buildBatteryPack(batcell);
batpack = polyfitBatterySOC(batpack);

%% Loading Electric Motor           -- ~1 min
load('em_155kW_3000rpm.mat');

% Create look-up table for em_eta
lut_eta_em = em_eta_table(em);

%% Loading NOx and NO -map          -- ~30 sec
load('EO_NOx_map.mat');

% Define max NO emission in ppm
max_NO_ppm  = 2000;

% Create look-up tables
lut_NO2NO   = emission_lut_NO2NO();
lut_NO      = emission_lut_NO(max_NO_ppm);

%% Build param

% Load steady-state setpoint from the LD2 engine map
load('liu_diesel_2_params.mat');

ice_param.lut_NO2NO = lut_NO2NO;
ice_param.lut_NO    = lut_NO;

% Define param struct for liu_diesel_2
param.ld2 = struct( ... 
    'ice_param',  ice_param ...
    );

% Define param struct for electric motor
param.em = struct( ...
    'batpack', batpack, ...
    'lut_eta_em', lut_eta_em ...
    );


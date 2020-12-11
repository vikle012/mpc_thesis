function param = init_hev_pwl(param)
%% VARIABLES
load('batcell_2_3Ah.mat', 'batcell');

batcell.NoCells.series      = 197;      % Cells in series
batcell.NoCells.parallel    = 8;        % Cells in parallel
batcell.SOC                 = 0.375;    % Initially charged to 50%
batcell.SOC_max             = 0.5;      % Max SOC [0.2 - 0.8]
batcell.SOC_min             = 0.25;     % Min SOC [0.2 - 0.8]
max_NO_ppm                  = 2000;     % Approximately largest NO value [ppm]

%% BATTERY

batpack = buildBatteryPack(batcell);
batpack = polyfitBatterySOC(batpack);

%% ELECTRIC MOTOR
load('em_155kW_3000rpm.mat', 'em');

% create look-up table for em_eta only if not done already
if ~isfield(param, 'em')
    lut_eta_em = em_eta_table(em);
else
    lut_eta_em = param.em.lut_eta_em;
end

%% NOx AND NO LOOK-UP TABLES
load('EO_NOx_map.mat', 'EO_NOx_map');

% Create look-up tables for NO and NO2/NO only if not already done
if ~isfield(param, 'ld2')
    lut_NO2NO = emission_lut_NO2NO();
    lut_NO    = emission_lut_NO(max_NO_ppm);
else
    lut_NO2NO = param.ld2.lut_NO2NO;
    lut_NO    = param.ld2.lut_NO;
end

%% BUILD PARAM
% load steady-state setpoint from the LD2 engine map
load('ld2_pwl_param.mat', 'pwl_param');
param.ld2 = pwl_param;

% add data
param.ld2.lut_NO2NO = lut_NO2NO;
param.ld2.lut_NO    = lut_NO;
param.em.lut_eta_em = lut_eta_em;
param.batpack       = batpack;
%param.ld2.J         = 4.5;

%% MAX AND MIN VALUES
% max and min states and controls
param.ld2.constr.x_max = [pwl_param.constr.x_max(1:4); ...  % ld2 states
    1; ...                                  % NOx state
    2400; ...                               % Engine speed max
    batpack.SOC_max; ...                    % SOC state
    pwl_param.constr.x_max(5); ...          % ld2 control
    1; ...                                  % ld2 control
    pwl_param.constr.x_max(7:end); ...      % ld2 control
    1500];                                  % M_em control
param.ld2.constr.x_min = [pwl_param.constr.x_min(1:4); ...  % ld2 states
    0; ...                                  % NOx state
    400; ...                                % Engine speed min
    batpack.SOC_min; ...                    % SOC state
    pwl_param.constr.x_min(5:end); ...      % ld2 control
    -1500];                                 % M_em control

% max and min derivative of controls
param.ld2.constr.u_max = [pwl_param.constr.u_max; ...   % ld2 max control
    30000];                                  % M_em max
param.ld2.constr.u_min = [pwl_param.constr.u_min; ...   % ld2 min control
    -30000];                                 % M_em min

%% NORMALIZATION
% states and control
param.ld2.ocp.state_norm = [pwl_param.ocp.state_norm(1:4); ... % ld2 states
    1; ...                                % NOx state norm
    2400; ...                               % Engine speed norm
    1; ...                                  % SOC state norm
    pwl_param.ocp.state_norm(5:end); ...  	% ld2 control norm
    3000];                                  % M_em control norm
% derivative of controls
param.ld2.ocp.control_norm = [pwl_param.ocp.control_norm; ... % ld2 control
    30000];                               	% M_em

end

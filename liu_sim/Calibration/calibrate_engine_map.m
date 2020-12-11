function engine_map = calibrate_engine_map(N_speed, N_torque, ice_param)

efficiency_matrix = nan(N_torque, N_speed);
state_calibration = cell(N_torque, N_speed);
control_calibration = cell(N_torque, N_speed);
pmep = nan(N_torque, N_speed);

% Engine speed range
speed_range = linspace(ice_param.constr.N_ice_min, ice_param.constr.N_ice_max, N_speed);

v0 = struct;
v0.X = ice_param.ocp.x0;     % x0 from the benchmark optimal control problem
v0.U = ice_param.ocp.u0; % Reasonable values
v0.P = NaN;          % No free optimzation parameters

% Min torque
torque_min = zeros(length(speed_range), 1);
for k=1:length(speed_range)
    N_ice = speed_range(k);
    cap = min_torque_problem(N_ice, ice_param);
    capSol = yopSolveSpp(cap, v0);
    [~, ~, signals] = liu_diesel_2(capSol.X, capSol.U, N_ice, ice_param);
    torque_min(k) = signals.M_ice;
end

% Min torque
torque_max = zeros(length(speed_range), 1);
for k=1:length(speed_range)
    N_ice = speed_range(k);
    cap = max_torque_problem(N_ice, ice_param);
    capSol = yopSolveSpp(cap, v0);
    [~, ~, signals] = liu_diesel_2(capSol.X, capSol.U, N_ice, ice_param);
    torque_max(k) = signals.M_ice;
end

% torque_grid = nan(N_torque, N_speed);
% speed_grid = nan(N_torque, N_speed);
torque_grid = 10*ones(N_torque, N_speed);
speed_grid = 10*ones(N_torque, N_speed);

% Calculate calibration
for k=1:N_speed
    N_ice = speed_range(k);
    torque_grid(:, k) = linspace(torque_min(k)+1, torque_max(k)-1, N_torque); % Offset min/max value to ease optimization
    speed_grid(:, k) = N_ice;
    for n=1:N_torque   
        M_ice = torque_grid(n, k);
        cap = min_fuel_calibration(N_ice, M_ice, ice_param);
        capSol = yopSolveSpp(cap, v0);
        [~, ~, signals] = liu_diesel_2(capSol.X, capSol.U, N_ice, ice_param);
        efficiency_matrix(n, k) = signals.eta_f;
        state_calibration{n, k} = capSol.X;
        control_calibration{n, k} = capSol.U;
        pmep(n, k) = capSol.X(3) - capSol.X(2);
    end
end

% Engine limits
% Pice1 = ice_param.cPice(1)*power(speed_range,2)+ice_param.cPice(2)*speed_range+ice_param.cPice(3);
% Pice2 = ice_param.cPice(4)*power(speed_range,2)+ice_param.cPice(5)*speed_range+ice_param.cPice(6);
% power_ulim = [Pice1(Pice1<Pice2) Pice2(Pice2<=Pice1)];


% Save to map-struct
rpm_range = speed_range;
engine_map = struct;
engine_map.speed_range = speed_range;
engine_map.rpm_range = rpm_range;
engine_map.speed_grid = speed_grid;
engine_map.rpm_grid = speed_grid;
engine_map.torque_grid = torque_grid;
engine_map.efficiency_grid = efficiency_matrix;
% engine_map.power_upper_lim = power_ulim;
engine_map.state_calibration = state_calibration;
engine_map.control_calibration = control_calibration;
% engine_map.torque_upper_lim = power_ulim./speed_range;
engine_map.pmep = pmep;
end
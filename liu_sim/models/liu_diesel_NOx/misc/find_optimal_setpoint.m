function x0 = find_optimal_setpoint(N_ice, M_ice, engine_param)

cap = engine_min_fuel_calibration(N_ice, M_ice, engine_param);
v0 = struct; 
v0.X = engine_param.ocp.x0; 
v0.U = zeros(3,1);
v0.P = NaN; 
capSol = yopSolveSpp(cap, v0);
x0 = capSol.X;

end
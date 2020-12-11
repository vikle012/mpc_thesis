function x0 = calc_steadystate_setpoint(s0, v_init, lut, truck_param, print_setpoint)

N_ice0 = 1200;
w_ice0 = N_ice0*pi/30;
M_ice0 = 1000;

% Find guess for engine state
x0_engine = find_optimal_setpoint(N_ice0, M_ice0, truck_param.engine);
% pause(0.1)

% Initial guess truck setpoint
gamma0 = w_ice0/(2*pi)/(v_init*truck_param.r_w)/truck_param.i_f;
Ek0 = truck_param.m_v * v_init^2 / 2;
x0_guess = [x0_engine; Ek0; 0; gamma0];
u0 = zeros(4,1);

v0sp = struct;
v0sp.X = x0_guess;
v0sp.U = u0; 
v0sp.P = NaN;         

% Optimal setpoint entire system
cap = truck_stationary_min_fuel(v_init, s0, truck_param, lut);
capSol = yopSolveSpp(cap, v0sp);

x0 = capSol.X;

if print_setpoint
    fprintf('\n\n')
    disp '------ Initial Setpoint ------'
    fprintf([...
        'p_cac: %d \n',...
        'p_im: %d \n',...
        'p_em: %d \n',...
        'w_tc: %d \n',...
        'u_f: %f \n',...
        'u_thr: %f \n',...
        'u_wg: %f \n',...
        'N_ice: %f \n',...
        'v: %f \n',...
        'gamma: %f \n'], ...
        x0(1), x0(2), x0(3), x0(4), x0(5), x0(6), x0(7), ...
        sqrt(2*x0(8)/truck_param.m_v)/truck_param.r_w*x0(10)*truck_param.i_f*30/pi, ...
        sqrt(2*x0(8)/truck_param.m_v)*3.6, x0(10));
end

end
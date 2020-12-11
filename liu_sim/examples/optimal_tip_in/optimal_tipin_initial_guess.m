function v0 = optimal_tipin_initial_guess(x0, u0, N_ice_opt, pwl_param)
% Control indices, for reabability
[u_f, u_thr, u_wg] = enum(3);
    
% Add step to control signal
rel_step = 100;
u_step = [u0(u_f)+rel_step; u0(u_thr:u_wg)];

% Simulate step
tf = 5;
sim_model = @(t,x) liu_diesel_2(x, u_step, N_ice_opt, pwl_param);
opts = odeset('RelTol', 1e-9, 'AbsTol', 1e-9);
[t_sim, x_sim] = ode15s(sim_model, [0, tf], x0, opts);

% Simulated control signal
u_sim = repmat(u_step', length(t_sim), 1);

% pwl formulation
x_guess = [x_sim, u_sim];
u_guess = repmat([0 0 0], length(t_sim), 1);

% Create guess object
v0 = yopInitialGuess(tf, t_sim, x_guess, u_guess, 0);
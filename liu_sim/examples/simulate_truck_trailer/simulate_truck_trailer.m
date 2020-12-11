load ld2_eng_map.mat
load truck_trailer_param.mat
load road_Sodertalje_Norrkoping.mat

%% Find steady-state starting point

s0 = 0; % Start position
v0 = 80/3.6; % Initial velocity
road_lut = load_lut(s0, 1e4, 'flat', road.x_distanceVector, road.k_slopeVector); % Road topography
x0 = calc_steadystate_setpoint(s0, v0,  road_lut, truck_param, true); % s-s setpoint

%% Simulate a step in fuel injection

fuel_step_size = 50;
sf = 1000; % Distance to simulate
u0 = zeros(4,1);
model = @(t, x) truck_trailer_ds(t, x, u0, truck_param, road_lut);
x0_sim = x0;
x0_sim(5) = x0(5)+fuel_step_size;
opts = odeset('RelTol', 1e-9, 'AbsTol', 1e-9);
[s_sim, x_sim] = ode15s(model, [0 sf], x0_sim, opts);
u_sim = repmat(u0', length(s_sim), 1);

%% Plot results
truck_param.lambda_min = truck_param.engine.lambda_min; % For plotting
truck_param.BSR_min = truck_param.engine.BSR_min;
truck_param.BSR_max = truck_param.engine.BSR_max;

vehicle = @(t, x, u) truck_trailer_ds(t, x, u, truck_param, road_lut);
sim_signals = vehicle_internal_signals(vehicle, s_sim, x_sim, u_sim, 's [m]');

% Engine plots
example_plots(sim_signals, engine_map)

% Plot line width
lw = 2;
% Subplot plot function
plot_sig = @(sig_name) plot_engine_signal(sig_name, sim_signals, lw);

% States
LD2_figure();
ax1 = subplot_hg(411);
plot_sig('v_kmh')
ax2 = subplot_hg(412);
plot_sig('R_a')
ax3 = subplot_hg(413);
plot_sig('R_r')
ax4 = subplot_hg(414);
plot_sig('R_g')
linkaxes([ax1,ax2,ax3,ax4],'x')
linkaxes([ax2,ax3,ax4],'xy')
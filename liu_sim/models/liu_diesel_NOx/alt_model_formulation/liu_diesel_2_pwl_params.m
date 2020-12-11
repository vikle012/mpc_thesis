load('liu_diesel_2_params.mat');

pwl_param = ice_param;
pwl_param.constr.x_min = [ice_param.constr.x_min; ice_param.constr.u_min];
pwl_param.constr.x_max = [ice_param.constr.x_max; ice_param.constr.u_max];
cycles_per_sec = 2400/60/2;
t_c2c = 1/cycles_per_sec;
du_f_max = 280/t_c2c;
du_thr_max = 20; % [Hz]
du_wg_max = 20; % [Hz]
pwl_param.constr.u_max = [du_f_max; du_thr_max; du_wg_max];
pwl_param.constr.u_min = [-du_f_max; -du_thr_max; -du_wg_max];
pwl_param.ocp.state_norm = [ice_param.ocp.state_norm; ice_param.ocp.control_norm];
pwl_param.ocp.control_norm = pwl_param.constr.u_max;
pwl_param.ocp.x0 = [ice_param.ocp.x0; ice_param.ocp.u0];
pwl_param.ocp.u0 = zeros(3,1);

save ld2_pwl_param pwl_param
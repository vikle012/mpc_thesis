function xdot = validation_model_wrapper(t, x, t_ip, u_ip, engine_model)

u = interp1q(t_ip, u_ip, t);
xdot = engine_model(x, u');

end


function [ Me, aux ] = engine_torque(Ne, p_im, p_em, u_f, ice_param)
% [ Me, aux ] = engine_torque(Ne,p_im,p_em,u_f,param)

Ne_rps = Ne/60;    % -> rps
w_ice  = Ne*pi/30; % -> rad/s

eta_igch = eta_factor(u_f, w_ice, ice_param);
eta_ig = eta_igch .* ( 1 - 1/( ice_param.r_c^(ice_param.gamma_cyl - 1)) );
W_pump = ice_param.V_D .* (p_em - p_im);
W_ig = ice_param.q_HV .* eta_ig .* u_f * 1e-6 * ice_param.n_cyl;
W_fric = ice_param.V_D.*(ice_param.c_fric(1) + ice_param.c_fric(2)*Ne_rps./10 + ice_param.c_fric(3)*(Ne_rps./10).^2);
Me = (W_ig - W_fric - W_pump)./(4*pi);

% Auxiliary output
aux = struct;
aux.eta_ig = eta_ig;
aux.W_pump = W_pump;
aux.W_ig = W_ig;
aux.W_fric = W_fric;
aux.M_pump = W_pump/4/pi;
aux.M_ig = W_ig/4/pi;
aux.M_fric = W_fric/4/pi;
aux.eta_igch = eta_igch;

end

function eta_igch = eta_factor(u_f, w_ice, param)
eta_igch = param.eta_igch(3).*(u_f./w_ice).^2 + param.eta_igch(2).*(u_f./w_ice) + param.eta_igch(1);

end

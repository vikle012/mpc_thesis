function [ W_a ] = W_cylinder( eta_vol, p_im, T_im, N_e, param )

W_a = eta_vol .* p_im .* N_e.*pi./30.* param.V_D ./ ...
       (4*pi * param.R_a .* T_im);


end


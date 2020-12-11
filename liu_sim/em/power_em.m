%% Returns the power consumed by the electric motor given torque
function P_em_electric = power_em(M_em, w_em, eta_em)

    P_em_electric = M_em .* w_em .* eta_em .^ (-(M_em ./ sqrt(M_em .^ 2)));
    
end
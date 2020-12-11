function T_out = turbine_T(Pi_turb, T_em, eta_tm, param)
    T_out = T_em + (T_em.*eta_tm) .* (Pi_turb .^ ((param.gamma_cyl - 1)/param.gamma_cyl) - 1);
    %T_out = T_em .* (Pi_turb).^((param.gamma_cyl - 1)./param.gamma_cyl);
end


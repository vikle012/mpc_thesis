%% Adiabatic expansion through orifice (wastegate)
function T_wg = wastegate_T(Pi_turb, T_em, gamma)
    VV = exp(log(Pi_turb)./gamma);   % V1/V2
    T_wg = (Pi_turb ./ VV) .* T_em;
end


function U_b = battery_voltage(P_em_electric, U_oc, batpack)
    U_b = (U_oc./2) + sqrt(((U_oc.^2)./4) - (P_em_electric .* batpack.R));
end


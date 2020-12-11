function I_b = battery_current(P_em_electric, U_b)
    I_b = P_em_electric ./ U_b;
end


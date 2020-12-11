%% Power drained or gain in battery
function P_oc = battery_power(U_oc, I_b)
    P_oc = U_oc .* I_b;
end


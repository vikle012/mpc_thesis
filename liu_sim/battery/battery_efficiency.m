%% Calculates the efficiency in the battery as a result of series resistor losses
function eta_b = battery_efficiency(P_b, P_oc)
% PROBLEM OM P_b = 0!  (divide by 0)

eta_b = (P_b ./ P_oc) .^ ((P_b ./ sqrt(P_b .^ 2)));
end


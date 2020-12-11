%% Returns coefficients for SOC to Uoc polynom
function batpack = polyfitBatterySOC(batpack)
    order = 3;
    working_area = 0.2:0.001:0.8; % Hardcoded max range
    voltage = interp1(batpack.socix, batpack.Voc, working_area, 'pchip');
    [batpack.polyfitCoefficients] = polyfit(working_area, voltage, order);
end


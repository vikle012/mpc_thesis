%% Calculate steady state fuel flow function

load('ld2_eng_map.mat')

step = diff(engine_map.rpm_range);

range = 500:step(1):2400;
fuel = zeros(1,length(range));
for Ni = 1:length(range)
    N = range(Ni);
    
    % retrieve values for steady state operation for given torque and speed
    [~, N_index_init] = min(abs( engine_map.rpm_range - N ));
    [~, M_index_init] = min(abs( engine_map.torque_grid(:, N_index_init) - M_init ));
    
    N_calibration = engine_map.rpm_range(N_index_init);
    M_calibration = engine_map.torque_grid(N_index_init, N_index_init);
    fuel(Ni) = engine_map.control_calibration{M_index_init, N_index_init}(1);
end

plot(range,fuel); hold on;
P = polyfit(range,fuel,2);
fuel_fit = polyval(P,range);
plot(range,fuel_fit,'r');







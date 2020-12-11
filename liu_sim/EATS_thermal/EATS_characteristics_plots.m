%% Plot script for analyzing EATS characteristics
% Load data
load ld2_NOx_EATS_0_1800_K1d4.mat
run test.m
%% Plot WHTC deNOx performance with T_aSCR from filtered vs. unfiltered engine out temperature
T_min_vec = 200;

deNOx = deNOx_fun(opt_signals.T_aSCR.val, T_min_vec);
deNOx_filtered = deNOx_fun(signals.T_aSCR-273, T_min_vec);

figure()
plot(opt_signals.t_vec.val, deNOx)
hold on
plot(t, deNOx_filtered)

%% Plot WHTC deNOx performance for different light-off temperatures
T_min_vec = [180:5:200];
deNOx = deNOx_fun(opt_signals.T_aSCR.val, T_min_vec);
figure()
for i = 1:length(T_min_vec)
plot(opt_signals.t_vec.val,deNOx(i,:))
hold on
end
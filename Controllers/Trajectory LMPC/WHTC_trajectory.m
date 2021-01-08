max_torque = 2100;
max_speed = 2000;

load WHTC_data

M_e_input = max_torque * torque/100;
n_e_input = max_speed * speed/100;

subplot(2,1,1)
plot(time, M_e_input)
title("Engine torque, M_e [Nm]")

subplot(2,1,2)
hold on
plot(time, n_e_input)
plot([time(1) time(end)], [500 500], 'r--')
plot([time(1) time(end)], [2000 2000], 'r--')
title("Engine rotational speed, n_e [rpm]")

% use find_trajectory
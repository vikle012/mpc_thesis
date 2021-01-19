% max_torque = 2100;
% max_speed = 2000;

load WHTC_data

n_idle = 500;
n_lo = 815;
n_pref = 1418;
n_95 = 1966;
n_hi = 2301;

n_e = 2.0327*speed/100*(0.45*n_lo + 0.45*n_pref + 0.1*n_hi - n_idle) + n_idle;

figure(1)
hold on
plot(time, n_e)
plot([time(1) time(end)], [500 500], 'r--')
plot([time(1) time(end)], [2000 2000], 'r--')

% M_e_input = max_torque * torque/100;
% n_e_input = max_speed * speed/100;
% 
% subplot(2,1,1)
% plot(time, M_e_input)
% title("Engine torque, M_e [Nm]")
% 
% subplot(2,1,2)
% hold on
% plot(time, n_e_input)
% plot([time(1) time(end)], [500 500], 'r--')
% plot([time(1) time(end)], [2000 2000], 'r--')
% title("Engine rotational speed, n_e [rpm]")

% use find_trajectory
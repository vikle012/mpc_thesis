load 'denormWHTC_values'

new_t = [time(1):0.01:time(end)]';
new_torque = interp1(time, actual_torque, new_t, 'pchip');
new_speed = interp1(time, actual_speed, new_t, 'pchip');

h = figure;
hold on
subplot(2,1,1)
plot(new_t, new_torque, '.');
ylabel('Engine torque [Nm]')

subplot(2,1,2)
plot(new_t, new_speed, '.');
ylabel('Engine speed [rpm]')
xlabel('Time [s]')

% set(h,'Units','Inches');
% pos = get(h,'Position');
% set(h,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
% print(h, 'Figures/Results/WHTC_evaluationcycle','-dpdf','-r0')
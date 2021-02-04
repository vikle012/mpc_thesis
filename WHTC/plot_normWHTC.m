%% Load and plot data

data = readtable("WHTC.txt");
load('WHTC_data.mat', 'time', 'speed', 'torque');

t = str2double(table2array(data(2:end, 1)));
sp = str2double(table2array(data(2:end, 2)));
to = str2double(table2array(data(2:end, 3)));

% Comparison between WHTC.txt and WHTC_data.mat
h = figure;
subplot(2,1,1)
hold on
plot(time, torque)
plot(t, to, 'r--')
ylabel('Normalized torque [%]')
xlabel('Time [s]')

subplot(2,1,2)
hold on
plot(time, speed)
plot(t, sp, 'r--')
ylabel('Normalized engine speed [%]')
xlabel('Time [s]')

%% Export as PDF

% set(h,'Units','Inches');
% pos = get(h,'Position');
% set(h,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
% print(h,'Figures/WHTC','-dpdf','-r0')
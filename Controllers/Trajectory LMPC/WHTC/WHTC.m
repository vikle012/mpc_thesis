clc
clear all
close all

data = readtable("WHTC.txt");

t = str2double(table2array(data(2:end, 1)));
sp = str2double(table2array(data(2:end, 2)));
to = str2double(table2array(data(2:end, 3)));

load WHTC_data

h = figure;
subplot(2,1,1)
hold on
% plot(t, to)
plot(time, torque)
ylabel('Normalized torque [%]')
xlabel('Time [s]')

subplot(2,1,2)
hold on
% plot(t, sp)
plot(time, speed)
ylabel('Normalized engine speed [%]')
xlabel('Time [s]')

% suptitle('World Harmonized Transient Cycle')

%% Export as PDF
set(h,'Units','Inches');
pos = get(h,'Position');
set(h,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
print(h,'Figures/WHTC','-dpdf','-r0')
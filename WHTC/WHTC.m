clc
clear all
close all

data = readtable("WHTC.txt");

t = str2double(table2array(data(2:end, 1)));
speed = str2double(table2array(data(2:end, 2)));
torque = str2double(table2array(data(2:end, 3)));

figure
subplot(2,1,1)
plot(t, torque)
ylabel('Normalized torque [%]')

subplot(2,1,2)
plot(t, speed)
ylabel('Normalized engine speed [%]')
xlabel('Time [s]')

suptitle('World Harmonized Transient Cycle')

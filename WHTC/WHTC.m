clc
clear

data = readtable("WHTC.txt");

t = str2double(table2array(data(2:end, 1)));
speed = str2double(table2array(data(2:end, 2)));
torque = str2double(table2array(data(2:end, 3)));

figure
hold on
plot(t, speed, 'b')
plot(t, torque, 'r')
legend(["Speed", "Torque"], 'location', 'northeast')

clc
clear all
close all
load('em_155kW_3000rpm.mat');
%%
tic
lut_em = em_ploss_table(em);
toc
%%  For plotting the interpolated function "lut"
tic
wix = linspace(500*pi/30, 2400*pi/30, 200);
Tix = linspace(min(em.Tix), max(em.Tix), 200);

for j = 1:length(wix)
    for i = 1:length(Tix)
        em_model_map.wix(i,j) = wix(j);
        em_model_map.Tix(i,j) = Tix(i);
        em_model_map.eta(i,j) = full(lut_em([wix(j) Tix(i)])); 
    end
    j
end

em_model_map.Tmax = interp1(em.wix, em.Tmax, em_model_map.wix(1,:)', 'linear');
em_model_map.Tmin = interp1(em.wix, em.Tmin, em_model_map.wix(1,:)', 'linear');
em_model_map.wix = em_model_map.wix .* 30 ./pi;
% save('liu_sim/em/em_efficiency.mat','em_model_map'); For use in optimal
% control

%%  Plotting

line_grid = [0:0.1:0.7, 0.7:0.005:0.9, 0.9:0.001:1];    % eta
line_grid = [0 500 1000:1000:24000];    % Ploss
line_text_size = 8;
axis_text_size = 8;
text_spacing = 550;
xaxis_text = 'Motor speed [rpm]';
yaxis_text = 'Motor torque [Nm]';
title_text = 'Electric engine power loss map';

LD2_figure(); hold on;
render_em_map(...
    em_model_map, ...
    line_grid, ...
    line_text_size, ...
    axis_text_size, ...
    text_spacing, ...
    xaxis_text, ...
    yaxis_text, ...
    title_text ...
);

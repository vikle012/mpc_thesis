%%% Executable script used to inspect the lookup table of the NOx map.%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc
clear all
close all
load 'ld2_eng_map.mat'
%% Create lookup table
tic
lut_NOx = NOx_table();
toc
%%  For plotting the interpolated function "lut"
tic
% Add the negative friction torque to motor map
Torque_grid_min = [(-3.356001893629823e-05 .* engine_map.rpm_range .^ 2 ...
     + 0.002507880152583 .* engine_map.rpm_range ...
     - 76.058728963433370);
     engine_map.torque_grid];

wix = linspace(engine_map.speed_range(1), engine_map.speed_range(end), 200);
Tix = linspace(min(min(Torque_grid_min)), max(max(Torque_grid_min)), 200);

for j = 1:length(wix)
    for i = 1:length(Tix)
        NOx_map.wix(i,j) = wix(j);
        NOx_map.Tix(i,j) = Tix(i);
        NOx_map.NOx(i,j) = full(lut_NOx([wix(j) Tix(i)]));
    end
    j
end

NOx_map.Tmax = interp1(engine_map.speed_range, engine_map.torque_grid(end,:), NOx_map.wix(1,:)', 'linear');
NOx_map.Tmin = interp1(engine_map.rpm_range, (-3.356001893629823e-05 .* engine_map.rpm_range .^ 2 ...
     + 0.002507880152583 .* engine_map.rpm_range ...
     - 76.058728963433370), NOx_map.wix(1,:)', 'linear');
NOx_map.wix = NOx_map.wix ;
%save('NOx_model_map.mat','NOx_map'); %For use in optimal
% control

%%  Plotting
line_grid = [0.000000000000001:0.01:0.4];
line_text_size = 8;
axis_text_size = 8;
text_spacing = 550;
xaxis_text = 'Motor speed [rpm]';
yaxis_text = 'Motor torque [Nm]';
title_text = 'Engine out NOx emissions [g/s]';

LD2_figure(); hold on;
render_NOx_map(...
    NOx_map, ...
    line_grid, ...
    line_text_size, ...
    axis_text_size, ...
    text_spacing, ...
    xaxis_text, ...
    yaxis_text, ...
    title_text ...
);

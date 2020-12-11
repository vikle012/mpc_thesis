%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Plot electric motor efficiency map %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

load('em_155kW_3000rpm.mat');
em.eta(em.eta < 0.01) = 0.01;
% Omformning för att matcha engine_map
em2.wix = repmat(em.wix', length(em.Tix),1) * 30/pi; % To rpm from rad/s
em2.Tix = repmat(em.Tix, 1,length(em.wix));
em2.eta = em.eta;
em2.Tmax = em.Tmax;
em2.Tmin = em.Tmin;

line_grid = [0, 0.2, 0.6, 0.8, 0.85, 0.87, 0.88, 0.89, 0.90, 0.905, 0.91];
line_grid = 0:0.1:0.9;
line_text_size = 8;
axis_text_size = 8;
text_spacing = 550;
xaxis_text = 'Speed [rpm]';
yaxis_text = 'Torque [Nm]';
title_text = 'Electric motor efficiency map';

LD2_figure(); hold on;
render_em_map(...
    em2, ...
    line_grid, ...
    line_text_size, ...
    axis_text_size, ...
    text_spacing, ...
    xaxis_text, ...
    yaxis_text, ...
    title_text ...
);


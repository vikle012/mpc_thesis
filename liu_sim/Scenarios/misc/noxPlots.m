function noxPlots(signals, engine_map)
% FUNCTION: Plot signals related to emissions
%
% SYNTAX: nowPlots(input1, input2);
%
% INPUTS:
%   input1 - Struct containing signals
%   input2 - Struct containing engine_map
%
% OUTPUTS:
%   none
%
% EXAMPLE:
%   noxPlots(opt_signals, engine_map);
%
% OTHER FILES REQUIRED:
%   .m files:
%       plot_engine_signal
%
%   .mat files:
%       no_model_map
%       no2_model_map
%
% SUBFUNCTIONS:
%   none
%
% Author: Hampus Andersson and Fredrik Andersson
% Mars 2018; Last revision: 17-April-2018

%------------- BEGIN CODE --------------
load('NOx_model_map.mat', 'NOx_map');
load('no_model_map.mat', 'no_model_map');
load('no2_model_map.mat', 'no2_model_map');

% Plot line width
lw = 2;

% Subplot plot function
plot_sig = @(sig_name) plot_engine_signal(sig_name, signals, lw);

%% Engine out emissions
LD2_figure();
ax1 = subplot_hg(211);
plot_sig('NOx_EO');
ax2 = subplot_hg(212);
plot_sig('NOx_tot');
linkaxes([ax1 ax2], 'x')

%% NOx map plot
% Settings
y_min   = -300;
y_max   = 2400;
x_min   = 500;
x_max   = 2400;
line_text_size  = 8;
axis_text_size  = 8;
text_spacing    = 550;
xaxis_text = 'Engine speed [rpm]';
yaxis_text = 'Engine torque [Nm]';

% Minium torque fitted polynomial
M_fric = (-3.356001893629823e-05 .* engine_map.rpm_range .^ 2 ...
    + 0.002507880152583 .* engine_map.rpm_range ...
    - 76.058728963433370);

% Zero torque line color
color_zero = [0.8 0.8 0.8];      % Grey
% Max and min torque line color
color_min_max = [1 0.5 0];  % Orange

line_grid_no = [0.000000000000001:0.01:0.4];
title_text_no = 'NOx emission map';

LD2_figure(); hold on;
render_NOx_map(...
    NOx_map, ...
    line_grid_no, ...
    line_text_size, ...
    axis_text_size, ...
    text_spacing, ...
    xaxis_text, ...
    yaxis_text, ...
    title_text_no ...
    );

% Maximum torque
fill([x_min engine_map.rpm_range x_max], ...
    [y_max engine_map.torque_grid(end,:) y_max], 'w');
plot(engine_map.rpm_range, engine_map.torque_grid(end,:), ...
    '-', 'Color', color_min_max, 'LineWidth', 2);

% Minimum torque
fill([engine_map.rpm_range x_max x_min], ...
    [M_fric y_min y_min], 'w');
plot(engine_map.rpm_range, M_fric, ...
    '-', 'Color', color_min_max, 'LineWidth', 2);

% Plot torque and engine speed
plot(signals.N_ice.val, signals.M_ice.val, 'rx');

ylim([y_min y_max]);
xlim([x_min x_max]);

% Line at torque = 0
l = line([x_min x_max], [0 0]);
l.Color = color_zero;
l.LineWidth = 1.5;
l.LineStyle = '--';


%% Engine out vs. tailpipe NOx

NOx_EO = zeros(length(signals.t_vec.val),1);
NOx_TP_Cu = zeros(length(signals.t_vec.val),1);
NOx_TP_Fe = zeros(length(signals.t_vec.val),1);

for i = 2:length(signals.NOx_TP_Cu.val)
    NOx_TP_Cu(i) = trapz(signals.t_vec.val(1:i), signals.NOx_TP_Cu.val(1:i)); 
    NOx_TP_Fe(i) = trapz(signals.t_vec.val(1:i), signals.NOx_TP_Fe.val(1:i)); 
    NOx_EO(i)    = trapz(signals.t_vec.val(1:i), signals.NOx_EO.val(1:i)); 
end

LD2_figure(); hold on;
plot(signals.t_vec.val, NOx_EO, 'k-', 'LineWidth', 2)
plot(signals.t_vec.val, NOx_TP_Cu, '-.', 'LineWidth', 2)
plot(signals.t_vec.val, NOx_TP_Fe, '--', 'LineWidth', 2)
legend('Engine out', 'CuZ', 'FeZ', 'Location', 'NorthWest');

grid on;
ylim([0 signals.NOx_tot.val(end) .* 1.05]);
xlim([signals.t_vec.val(1) signals.t_vec.val(end)]);

%% CuZ/FeZ catalyst activity & SCR temperature
LD2_figure(); hold on
ax1 = subplot_hg(211);
plot_sig('deNOx_Cu')
plot_sig('deNOx_Fe')
title('CuZ and FeZ deNOx activity')
legend('CuZ', 'FeZ')
ax2 = subplot_hg(212);
plot_sig('T_SCR3')
linkaxes([ax1, ax2], 'x')

%% Engine out & TP NOx bar graph
figure();
y_bar = [trapz(signals.t_vec.val, signals.NOx_EO.val); trapz(signals.t_vec.val, signals.NOx_TP_Cu.val); trapz(signals.t_vec.val, signals.NOx_TP_Fe.val)];
c = categorical({'Engine out', 'CuZ', 'FeZ'});
c = reordercats(c, {'Engine out', 'CuZ', 'FeZ'});
bar(c, y_bar)
ylabel('NOx [g]')
title('Engine out vs. tailpipe NOx for WHTC')

%% EATS temperatures
LD2_figure();
ax1 = subplot_hg(411);
plot_sig('T_atw_c')
ax2 = subplot_hg(412);
plot_sig('T_aDOC')
ax3 = subplot_hg(413);
plot_sig('T_aDPF')
ax4 = subplot_hg(414);
plot_sig('T_SCR3')
linkaxes([ax1,ax2,ax3,ax4],'x')
linkaxes([ax2,ax3],'xy')
%% SCR segment temperatures
LD2_figure(); hold on
plot_sig('T_SCR1')
plot_sig('T_SCR2')
plot_sig('T_SCR3')
plot_sig('T_SCR4')
plot_sig('T_SCR5')
title('SCR segment temperatures')
ylabel('Temperature [K]')
legend('S1','S2','S3','S4','S5', 'Location','NorthWest')
%------------- END OF CODE --------------
end
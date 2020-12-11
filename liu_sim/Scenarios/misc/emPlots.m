function emPlots(v0, signals)
% FUNCTION: Function for plotting all signals related to electric motor
%
% SYNTAX: emPlots(input1);
%
% INPUTS:
%   input1 - Struct containning initial guess
%   input2 - Struct containing all optimal signals
%
% OUTPUTS:
%   none
%
% EXAMPLE:
%   emPlots(v0, opt_signals);
%
% OTHER FILES REQUIRED:
%   .m files:
%       plot_engine_signal
%       LD2_figure
%       render_engine_map
%
%   .mat files:
%       em_155kW_3000rpm
%
% SUBFUNCTIONS:
%   none
%
% Author: Hampus Andersson and Fredrik Andersson
% Mars 2018; Last revision: 15-Mars-2018

%------------- BEGIN CODE --------------

load('em_155kW_3000rpm.mat', 'em');

% Plot line width
lw = 2;

% Subplot plot function
plot_sig = @(sig_name) plot_engine_signal(sig_name, signals, lw);

% SOC and Torque
LD2_figure(); hold on;
ax1 = subplot_hg(211);
plot_sig('SOC')
plot(v0.T + signals.t_vec.val(1), v0.X(:,7),'k', 'LineWidth', 1.5)
plot([signals.t_vec.val(1) signals.t_vec.val(end)], signals.parameters.batpack.SOC_min,'r--')
plot([signals.t_vec.val(1) signals.t_vec.val(end)], signals.parameters.batpack.SOC_max,'r--')
ylim([min(min([signals.SOC.val v0.X(:,7)']))*0.99 max(max([signals.SOC.val v0.X(:,7)'])*1.01)]);
legend('Optimal','Initial guess');
ax2 = subplot_hg(212);
plot_sig('M_em')
plot(v0.T + signals.t_vec.val(1), v0.X(:,11),'k', 'LineWidth', 1.5);
legend('Optimal','Initial guess');
linkaxes([ax1,ax2],'x')

% Power
LD2_figure();
ax1 = subplot_hg(211);
plot_sig('P_em_mech')
ax2 = subplot_hg(212);
plot_sig('P_em_elec')
ylim([min([signals.P_em_elec.val signals.P_em_mech.val]).*0.95 ...
    max([signals.P_em_elec.val signals.P_em_mech.val]).*1.05]);
linkaxes([ax1,ax2],'x')
linkaxes([ax1,ax2],'xy')

% Efficiencies
LD2_figure();
ax1 = subplot_hg(211);
plot_sig('eta_em')
ylim([0 100]);
ax2 = subplot_hg(212);
plot_sig('eta_b')
linkaxes([ax1,ax2],'x')

% Electrical
LD2_figure();
ax1 = subplot_hg(311);
plot_sig('U_oc')
ax2 = subplot_hg(312);
plot_sig('U_b')
ax3 = subplot_hg(313);
plot_sig('I_b')
linkaxes([ax1,ax2,ax3],'x')


%% Plot electric motor efficiency map 

em.eta(em.eta < 0.01) = 0.01;
% Omformning för att matcha engine_map
em2.wix = repmat(em.wix', length(em.Tix),1) * 30/pi; % To rpm from rad/s
em2.Tix = repmat(em.Tix, 1,length(em.wix));
em2.eta = em.eta;
em2.Tmax = em.Tmax;
em2.Tmin = em.Tmin;

line_grid = [0, 0.2, 0.6, 0.8, 0.85, 0.87, 0.88, 0.89, 0.90, 0.905, 0.91];
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
plot(signals.N_ice.val, signals.M_em.val,'rx-');
% Engine map
% line_grid = [0:0.1:0.2, 0.3:0.02:0.4, 0.4:0.01:0.42, ...
%     0.43, 0.435, 0.44, 0.445, 0.45, 0.455, 0.46, 0.47,0.48,0.49];
% 
% line_text_size = 8;
% axis_text_size = 8;
% text_spacing = 550;
% xaxis_text = 'Engine speed [rpm]';
% yaxis_text = 'Engine torque [Nm]';
% title_text = 'LiU Diesel 2 map';
% 
% LD2_figure(); hold on;
% render_engine_map(...
%     engine_map, ...
%     line_grid, ...
%     line_text_size, ...
%     axis_text_size, ...
%     text_spacing, ...
%     xaxis_text, ...
%     yaxis_text, ...
%     title_text ...
%     );
% plot(signals.N_ice.val, signals.M_ice.val, 'r-x');

hold off;
%------------- END OF CODE --------------
end
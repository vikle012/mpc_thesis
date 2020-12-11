function ld2_NOx_plots(signals, engine_map, no_model_map, no2_model_map)

% Plot line width
lw = 2;

% Subplot plot function
plot_sig = @(sig_name) plot_engine_signal(sig_name, signals, lw);

LD2_figure();
plot_sig('NOx_tot')


% States
LD2_figure();
ax1 = subplot_hg(411);
plot_sig('p_cac')
ax2 = subplot_hg(412);
plot_sig('p_im')
ax3 = subplot_hg(413);
plot_sig('p_em')

ax4 = subplot_hg(414);
plot_sig('N_t')
linkaxes([ax1,ax2,ax3,ax4],'x')
linkaxes([ax1,ax2,ax3],'xy')

% Control
LD2_figure();
ax1 = subplot_hg(311);
plot_sig('u_f')
ax2 = subplot_hg(312);
plot_sig('u_thr')
ax3 = subplot_hg(313);
plot_sig('u_wg')
linkaxes([ax1,ax2,ax3],'x')
linkaxes([ax2,ax3],'xy')

% Cylinder
LD2_figure();
ax1 = subplot_hg(421);
plot_sig('M_ice')
ax3 = subplot_hg(423);
plot_sig('M_fric')
ax5 = subplot_hg(425);
plot_sig('M_pump')
ax7 = subplot_hg(427);
plot_sig('M_ig')
linkaxes([ax1,ax3,ax5,ax7],'x')

ax2 = subplot_hg(422);
plot_sig('N_ice')
ax4 = subplot_hg(424);
plot_sig('P_ice')
ax6 = subplot_hg(426);
plot_sig('phi')
ax8 = subplot_hg(428);
plot_sig('Lambda')
plot([signals.t_vec.val(1) signals.t_vec.val(end)], signals.parameters.lambda_min*[1 1],'--')
legend('\lambda', '\lambda_{min}','location','e')
linkaxes([ax1,ax2,ax3,ax4,ax5,ax6,ax7,ax8],'x')

% Turbocharger
LD2_figure();
ax1 = subplot_hg(421);
plot_sig('eta_comp')
ax2 = subplot_hg(422);
plot_sig('eta_tm')
ax3 = subplot_hg(423);
plot_sig('Pi_comp')
ax4 = subplot_hg(424);
plot_sig('turb_exp_rat')
ax5 = subplot_hg(425);
plot_sig('T_comp')
ax6 = subplot_hg(426);
plot_sig('BSR')
plot([signals.t_vec.val(1) signals.t_vec.val(end)], signals.parameters.BSR_max*[1 1])
plot([signals.t_vec.val(1) signals.t_vec.val(end)], signals.parameters.BSR_min*[1 1])
legend('BSR', 'BSR_{max}', 'BSR_{min}');
ax7 = subplot_hg(427);
plot_sig('W_comp')
ax8 = subplot_hg(428);
plot_sig('N_t')

linkaxes([ax1,ax2],'xy')
linkaxes([ax1,ax2,ax3,ax4,ax5,ax6,ax7,ax8],'x')

% Plot temperatures
LD2_figure();
ax1 = subplot_hg(311);
plot_sig('T_em')
ax2 = subplot_hg(312);
plot_sig('T_atw')
ax3 = subplot_hg(313);
plot_sig('T_tw_drop')
linkaxes([ax1,ax2,ax3],'x')
linkaxes([ax1,ax2],'xy')

% Engine map
line_grid = [0:0.1:0.2, 0.3:0.02:0.4, 0.4:0.01:0.42, ...
             0.43, 0.435, 0.44, 0.445, 0.45, 0.455, 0.46, 0.47,0.48,0.49];
          
line_text_size = 8;
axis_text_size = 8;
text_spacing = 550;
xaxis_text = 'Engine speed [rpm]';
yaxis_text = 'Engine torque [Nm]';
title_text = 'LiU Diesel 2 map';

LD2_figure(); hold on;
render_engine_map(...
    engine_map, ...
    line_grid, ...
    line_text_size, ...
    axis_text_size, ...
    text_spacing, ...
    xaxis_text, ...
    yaxis_text, ...
    title_text ...
    );
plot(signals.N_ice.val, signals.M_ice.val, 'r-x');
%%%%%%% Plot nox maps and path %%%%%%%


% NO-specific settings
line_grid_no = [0:100:2200]; % NO
title_text_no = 'NO emission map';
LD2_figure(); % Plot NO map
hold on;
render_nox_map(...
    no_model_map, ...
    line_grid_no, ...
    line_text_size, ...
    axis_text_size, ...
    text_spacing, ...
    xaxis_text, ...
    yaxis_text, ...
    title_text_no ...
    );
load('ld2_eng_map.mat');
%contour(engine_map.rpm_grid, engine_map.torque_grid, engine_map.efficiency_grid,'k','ShowText','on', 'LineWidth', 2)
plot(engine_map.rpm_range, engine_map.torque_grid(end,:), 'r-', 'LineWidth', 2)
plot(engine_map.rpm_range, (-2.9291e-05.*engine_map.rpm_range.^2 + 1.9111e-04.*engine_map.rpm_range -73.4988), 'r-', 'LineWidth', 2) % Drag torque
plot(signals.N_ice.val, signals.M_ice.val, 'r-x');

% NO2-specific settings
line_grid_no2 = [0:5:100]; % NO2
title_text_no2 = 'NO_2 emission map';
LD2_figure(); % Plot NO2 map
hold on;
render_nox_map(...
    no2_model_map, ...
    line_grid_no2, ...
    line_text_size, ...
    axis_text_size, ...
    text_spacing, ...
    xaxis_text, ...
    yaxis_text, ...
    title_text_no2 ...
    );
load('ld2_eng_map.mat');
%contour(engine_map.rpm_grid, engine_map.torque_grid, engine_map.efficiency_grid,'k','ShowText','on', 'LineWidth', 2)
plot(engine_map.rpm_range, engine_map.torque_grid(end,:), 'r-', 'LineWidth', 2)
plot(engine_map.rpm_range, (-2.9291e-05.*engine_map.rpm_range.^2 + 1.9111e-04.*engine_map.rpm_range -73.4988), 'r-', 'LineWidth', 2)
plot(signals.N_ice.val, signals.M_ice.val, 'r-x');
end
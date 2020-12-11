function whtcPlots(v0, signals, param)
% FUNCTION: Plotting how the optimal controls relate to WHTC
%
% SYNTAX: whtcPlots(input1, input2, input3);
%
% INPUTS:
%   input1 - Struct containing the initial guess
%   input2 - Struct containing all optimal signals
%   input3 - Struct containing all parameters
%   input4 - 'ld2_NOx' or 'hev' depending on mode
%
% OUTPUTS:
%   none
%
% EXAMPLE:
%   whtcPlots(v0, opt_signals, param, 'hev');
%
% OTHER FILES REQUIRED:
%   .m files:
%       none
%
%   .mat files:
%       none
%
% SUBFUNCTIONS:
%   none
%
% Author: Hampus Andersson and Fredrik Andersson
% Mars 2018; Last revision: 15-Mars-2018

%------------- BEGIN CODE --------------

t0 = signals.t_vec.val(1);
tf = signals.t_vec.val(end);



% ENGINE SPEED
figure(); hold on;
% plot([t0+1; param.WHTC.time(t0+1:ceil(tf))], ...
%     [param.WHTC.speed_scaled(1); param.WHTC.speed_scaled(t0+1:ceil(tf))], ...
%     'k', 'LineWidth', 1);
if numel(signals.x_vec.val(1,:)) == 24
    plot(v0.T+t0, v0.X(:,24), ...
        'g', 'LineWidth', 2)
    plot(signals.t_vec.val, signals.x_vec.val(:,24), ...
        'r--', 'LineWidth', 1.5);
else
    plot(v0.T+t0, v0.X(:,6), ...
        'g', 'LineWidth', 2)
    plot(signals.t_vec.val, signals.x_vec.val(:,6), ...
        'r--', 'LineWidth', 1.5);
end
legend( ...'WHTC', ...
    'Initial guess','Optimal');
title('Engine speed');
ylabel('N (rpm)');
xlabel('Time (s)');

switch numel(signals.x_vec.val(1,:))
    case 11 % HEV
        
        % Index in x_vec for u_f
        control_idx = 8;
        
        % TORQUE
        figure(); hold on;
        plot([t0; param.WHTC.time(t0+1:ceil(tf))], ...
            [param.WHTC.torque_scaled(1); param.WHTC.torque_scaled(t0+1:ceil(tf),1)], ...
            'k', 'LineWidth', 2);
        plot(t0:0.01:ceil(tf), full(param.WHTC.lut_M(t0:0.01:ceil(tf))), ...
            'g', 'LineWidth', 1);
        plot(signals.t_vec.val, signals.M_ice.val, ...
            'c--', 'LineWidth', 1.5);
        plot(signals.t_vec.val, signals.M_em.val, ...
            'b--', 'LineWidth', 1.5);
        plot(signals.t_vec.val, signals.M_ice.val + signals.M_em.val, ...
            'r--', 'LineWidth', 1.5);
        
        legend('WHTC normalized', 'WHTC lut', 'Diesel engine', 'Electric motor', 'Combined');
        title('Torque');
        ylabel('M (Nm)');
        xlabel('WHTC (s)');
        
    case 9 % ld2_NOx
        
        % Index in x_vec for u_f
        control_idx = 7;
        
        % TORQUE
        figure(); hold on;
        plot([t0; param.WHTC.time(t0+1:ceil(tf))], ...
            [param.WHTC.torque_scaled(1); param.WHTC.torque_scaled(t0+1:ceil(tf),1)], ...
            'k', 'LineWidth', 2);
        plot(t0:0.01:ceil(tf), full(param.WHTC.lut_M(t0:0.01:ceil(tf))), ...
            'g', 'LineWidth', 1);
        plot(signals.t_vec.val, signals.M_ice.val, ...
            'r--', 'LineWidth', 1.5);
        legend('WHTC normalized', 'WHTC lut', 'Optimal');
        title('Torque');
        ylabel('M (Nm)');
        xlabel('WHTC (s)');
        
    case 21 % ld2_NOx_EATS
        
        % Index in x_vec for u_f
        control_idx = 19;
        
        % TORQUE
        figure(); hold on;
        plot([t0; param.WHTC.time(t0+1:ceil(tf))], ...
            [param.WHTC.torque_scaled(1); param.WHTC.torque_scaled(t0+1:ceil(tf),1)], ...
            'k', 'LineWidth', 2);
        plot(t0:0.01:ceil(tf), full(param.WHTC.lut_M(t0:0.01:ceil(tf))), ...
            'g', 'LineWidth', 1);
        plot(signals.t_vec.val, signals.M_ice.val, ...
            'r--', 'LineWidth', 1.5);
        legend('WHTC normalized', 'WHTC lut', 'Optimal');
        title('Torque');
        ylabel('M (Nm)');
        xlabel('WHTC (s)');
        
    case 23 % hev_EATS
        
        % Index in x_vec for u_f
        control_idx = 20;
        
        % TORQUE
        figure(); hold on;
        %         plot([t0; param.WHTC.time(t0+1:ceil(tf))], ...
        %             [param.WHTC.torque_scaled(1); param.WHTC.torque_scaled(t0+1:ceil(tf),1)], ...
        %             'k', 'LineWidth', 2);
        %         plot(t0:0.01:ceil(tf), full(param.WHTC.lut_M(t0:0.01:ceil(tf))), ...
        %             'g', 'LineWidth', 1);
        plot(signals.t_vec.val, signals.M_ice.val, ...
            'c--', 'LineWidth', 1.5);
        plot(signals.t_vec.val, signals.M_em.val, ...
            'b--', 'LineWidth', 1.5);
        plot(signals.t_vec.val, signals.M_ice.val + signals.M_em.val, ...
            'r--', 'LineWidth', 1.5);
        
        legend( ...'WHTC normalized', 'WHTC lut',
            'Diesel engine', 'Electric motor', 'Combined');
        title('Torque');
        ylabel('M (Nm)');
        xlabel('WHTC (s)');
    case 24 % SPECIAL free_N
        control_idx = 20;
        
        % TORQUE
        figure(); hold on;
        plot(signals.t_vec.val, signals.M_ice.val, ...
            'c--', 'LineWidth', 1.5);
        plot(signals.t_vec.val, signals.M_em.val, ...
            'b--', 'LineWidth', 1.5);
        plot(signals.t_vec.val, signals.M_ice.val + signals.M_em.val, ...
            'r--', 'LineWidth', 1.5);
        
        legend('Diesel engine', 'Electric motor', 'Combined');
        title('Torque');
        ylabel('M (Nm)');
        xlabel('WHTC (s)');
end

% INITIAL GUESS u_f
figure(); hold on;
plot(v0.T+t0, v0.X(:,control_idx), ...
    'k', 'LineWidth', 1.5);
plot(signals.t_vec.val, signals.x_vec.val(:, control_idx), ...
    'r--');
plot(v0.T+t0, ones(1, length(v0.T)).*4, ...
    '.', 'color', [0.8, 0.8, 0.8]);
legend('Initial guess','Optimal');
title('Fuel injection');
ylabel('u_{f}');
xlabel('WHTC (s)');
%ylim([-25 param.ld2.constr.x_max(control_idx)]);

% INITIAL GUESS u_th AND u_wg
figure(); hold on;
plot(v0.T+t0, v0.X(:,control_idx+1),'k','LineWidth', 1.5);
plot(v0.T+t0, v0.X(:,control_idx+2),'k','LineWidth', 1.5);
plot(signals.t_vec.val, signals.x_vec.val(:,control_idx+1), 'r');
plot(signals.t_vec.val, signals.x_vec.val(:,control_idx+2), 'b');
legend('Throttle initial guess','Wastegate initial guess', ...
    'Optimal throttle','Optimal wastegate','Location', 'West');
title('Throttle and wastegate control');
xlabel('WHTC (s)');
ylim([-0.1 1.1]);

hold off;
%------------- END OF CODE --------------
end


% NAME: LO_conventional_tradeoff
%
% PURPOSE: Compare three cases optimized for fastest light-off.
%
% OTHER FILES REQUIRED:
%   .m files:
%       none
%
%   .mat files:
%       light_off_kallstart_EATS_ld2_NOx_0_248_K1d4
%       light_off_kallstart_100Wf_1_EATS_ld2_NOx_0_255_K1d4
%       light_off_kallstart_30procent_mer_torque_EATS_ld2_NOx_0_217_K1d4
%       
%       
% SUBFUNCTIONS:
%   none
%
% Author: Hampus Andersson and Fredrik Andersson
% May 2018; Last revision: 23-May-2018

%------------- BEGIN CODE --------------

%% Load runs

close all;
opt_signals1 = load_run('light_off_kallstart_EATS_ld2_NOx_0_248_K1d4',0); 
opt_signals2 = load_run('light_off_kallstart_100Wf_1_EATS_ld2_NOx_0_255_K1d4',0); 
opt_signals3 = load_run('light_off_kallstart_30procent_mer_torque_EATS_ld2_NOx_0_217_K1d4',0); 

%% Plot

figure(); hold on;
plot(opt_signals1.t_vec.val, opt_signals1.x_vec.val(:,14)-273);
plot(opt_signals2.t_vec.val, opt_signals2.x_vec.val(:,14)-273);
plot(opt_signals3.t_vec.val, opt_signals3.x_vec.val(:,14)-273);
legend('ld2 100%','ld2 100% Wf','ld2 120%');
title('Comparison Light-Off');
ylabel('SCR3 temperature (\circC)');
xlabel('Time (s)');

figure(); hold on;
plot(opt_signals1.t_vec.val, opt_signals1.T_atw_c.val);
plot(opt_signals2.t_vec.val, opt_signals2.T_atw_c.val);
plot(opt_signals3.t_vec.val, opt_signals3.T_atw_c.val);
legend('ld2 100%','ld2 100% Wf','ld2 120%');
title('Comparison Light-Off');
ylabel('Out-of-engine temperature (\circC)');
xlabel('Time (s)');

figure(); hold on;
plot(opt_signals1.t_vec.val, opt_signals1.x_vec.val(:,19));
plot(opt_signals2.t_vec.val, opt_signals2.x_vec.val(:,19));
plot(opt_signals3.t_vec.val, opt_signals3.x_vec.val(:,19));
legend('ld2 100%','ld2 100% Wf','ld2 130%');
title('Comparison Light-Off');
ylabel('Fuel injection (mg/cycle)');
xlabel('Time (s)');

%------------- END OF CODE --------------
% NAME: plot_EATS_temp_WG
%
% PURPOSE: Plot the EATS temperatures with wastegate position and flow
%          through engine.
%
% OTHER FILES REQUIRED:
%   .m files:
%       plot_engine_signal
%
%   .mat files:
%       none
%
% SUBFUNCTIONS:
%   none
%
% Author: Hampus Andersson and Fredrik Andersson
% June 2018; Last revision: 04-June-2018

%------------- BEGIN CODE --------------

switch numel(opt_signals.x_vec.val(1,:))
    case 23
        idx = 8;
    case 21
        idx = 7;
    otherwise
        warning('ERROR! Unknown index!');
end



lw = 2;
%close all;
%clc
% Subplot plot function
plot_sig = @(sig_name) plot_engine_signal(sig_name, opt_signals, lw);

% States
LD2_figure();
ax1 = subplot_hg(2,2,1);
plot_sig('W_tw')
hold on;
yyaxis right;
plot(opt_signals.t_vec.val, opt_signals.T_atw_c.val, 'r--', 'LineWidth', 2);
ylabel('Temp. after turb. and WG');
ax1 = subplot_hg(2,2,2);
plot_sig('u_wg');

%LD2_figure();
ax1 = subplot_hg(2,2,[3, 4]);
plot_sig('T_atw_c'); hold on;

for i = idx:idx+7
   plot(opt_signals.t_vec.val, opt_signals.x_vec.val(:,i)-273,'LineWidth',1.5) 
end %'NumColumnsMode','manual'
legend('EO','D1','D2','F1','F2','F3','S1','S2','S3','Orientation', 'vertical');

%------------- END OF CODE --------------



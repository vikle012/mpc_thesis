function hevPlots(signals)
% FUNCTION: Plots signals related to power split between diesel engine
%           and electric motor
%
% SYNTAX: hevPlots(input1);
%
% INPUTS:
%   input1 - Struct containing all optimal signals
%
% OUTPUTS:
%   none
%
% EXAMPLE:
%   hevPlots(opt_signals);
%
% OTHER FILES REQUIRED:
%   .m files:
%       plot_engine_signal
%       LD2_figure
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

% Plot line width
lw = 2;

% Subplot plot function
plot_sig = @(sig_name) plot_engine_signal(sig_name, signals, lw);

LD2_figure(); hold on;
ax1 = subplot_hg(411);
plot_sig('M_ratio')
ax2 = subplot_hg(412);
plot_sig('M_em_norm')
ax3 = subplot_hg(413);
ylim([-1 1]);
plot_sig('M_ice_norm')
ax4 = subplot_hg(414);
plot_sig('M_tot')
plot_sig('M_ice')
plot_sig('M_em')
title('ICE-, EM- and sum torque')
legend('Torque sum','ICE torque','EM torque')
linkaxes([ax1,ax2,ax3,ax4],'x')
linkaxes([ax2,ax3],'xy')


% plot(signals.t_vec.val, signals.M_ice+signals.M_em)
% plot(signals.t_vec.val, signals.M_ice)
% plot(signals.t_vec.val, signals.M_em)

hold off;
%------------- END OF CODE --------------
end


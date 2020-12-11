function EATS_plots(signals)
% FUNCTION: Description of purpose of function
%
% SYNTAX: EATS_plots(input1);
%
% INPUTS:
%   input1 - Description of input1
%
% OUTPUTS:
%   none
%
% EXAMPLE:
%   EATS_plots(opt_signals);
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
% April 2018; Last revision: 17-April-2018

%------------- BEGIN CODE --------------

% Plot line width
lw = 2;

% Subplot plot function
plot_sig = @(sig_name) plot_engine_signal(sig_name, signals, lw);

% Plot temperatures
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

LD2_figure(); hold on
ax1 = subplot_hg(211);
plot_sig('deNOx_Cu')
plot_sig('deNOx_Fe')
title('CuZ and FeZ deNOx activity')
legend('CuZ', 'FeZ')
ax2 = subplot_hg(212);
plot_sig('T_SCR3')
linkaxes([ax1, ax2], 'x')

figure();
y_bar = [trapz(signals.t_vec.val, signals.NOx_EO.val); trapz(signals.t_vec.val, signals.NOx_TP_Cu.val); trapz(signals.t_vec.val, signals.NOx_TP_Fe.val)];
c = categorical({'Engine out', 'CuZ', 'FeZ'});
c = reordercats(c, {'Engine out', 'CuZ', 'FeZ'});
bar(c, y_bar)
ylabel('NOx [g]')
title('Engine out vs. tailpipe NOx for WHTC')

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
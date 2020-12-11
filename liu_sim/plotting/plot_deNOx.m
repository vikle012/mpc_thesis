% NAME: plot_deNOx
%
% PURPOSE: Compares engine and SCR temperatures, deNOx perfomance etc.
% between two datasets.
%
% OTHER FILES REQUIRED:
%   .m files:
%       none
%
%   .mat files:
%    	data file 1, for example ('EATS_ld2_NOx_0_1800_K1d4.mat')
%    	data file 2, for example ('HEV_0_1800_K1d4_WG_ON_tol6_rel5_17000s_100Nm.mat')
%
% SUBFUNCTIONS:
%   none
%
% Author: Hampus Andersson and Fredrik Andersson
% Mars 2018; Last revision: 29-May-2018

%------------- BEGIN CODE --------------

% Load data 

data1 = load_run('EATS_ld2_NOx_0_1800_K1d4.mat');
data2 = load_run('HEV_0_1800_K1d4_WG_ON_tol6_rel5_17000s_100Nm.mat');

[deNOx_Cu1, deNOx_Fe1, deNOx_Cu2, deNOx_Fe2, signals1, signals2, t1, t2] = test_fun(data1, data2);

% Calculate tailpipe NOx: NOx_TP = NOx_EO*deNOx, for CuZ and FeZ catalysts
% Conventional data:
deNOx_vec_Cu1 = interp1(t1, deNOx_Cu1, data1.t_vec.val);
deNOx_vec_Fe1 = interp1(t1, deNOx_Fe1, data1.t_vec.val);
try
    NOx_TP_Cu1 = data1.NOx.val.*(1-deNOx_vec_Cu1);
    NOx_TP_Fe1 = data1.NOx.val.*(1-deNOx_vec_Fe1);
catch
    NOx_TP_Cu1 = data1.NOx_EO.val.*(1-deNOx_vec_Cu1);
    NOx_TP_Fe1 = data1.NOx_EO.val.*(1-deNOx_vec_Fe1);
end
% Hybrid data:
deNOx_vec_Cu2 = interp1(t2, deNOx_Cu2, data2.t_vec.val);
deNOx_vec_Fe2 = interp1(t2, deNOx_Fe2, data2.t_vec.val);
try
    NOx_TP_Cu2 = data2.NOx.val.*(1-deNOx_vec_Cu2);
    NOx_TP_Fe2 = data2.NOx.val.*(1-deNOx_vec_Fe2);
catch
    NOx_TP_Cu2 = data2.NOx_EO.val.*(1-deNOx_vec_Cu2);
    NOx_TP_Fe2 = data2.NOx_EO.val.*(1-deNOx_vec_Fe2);
end

NOx_TP_tot_Cu1 = trapz(data1.t_vec.val, NOx_TP_Cu1);
NOx_TP_tot_Fe1 = trapz(data1.t_vec.val, NOx_TP_Fe1);
NOx_TP_tot_Cu2 = trapz(data2.t_vec.val, NOx_TP_Cu2);
NOx_TP_tot_Fe2 = trapz(data2.t_vec.val, NOx_TP_Fe2);
%NOx_ratio1 = (1 - NOx_TP_tot1/data1.NOx_tot.val(end))*100;
%NOx_ratio2 = (1 - NOx_TP_tot2/data2.NOx_tot.val(end))*100;

%% Plot

% T_atw plots
m1 = movmean(data1.T_atw.val-273, 200); % Moving mean temps
m2 = movmean(data2.T_atw.val-273, 200);
figure(); hold on
plot(data1.t_vec.val, data1.T_atw.val - 273,':', data2.t_vec.val, data2.T_atw.val -273,':');
ax1 = gca;
ax1.ColorOrderIndex = 1;
plot(data1.t_vec.val, m1,'-.', 'LineWidth', 2)
%plot(data1.t_vec.val, m2, 'k', 'LineWidth', 2)
plot(data1.t_vec.val, m2,'--', 'LineWidth', 2)
ylim([0 750]);
title('Conventional and hybrid temperatures and moving means')
xlabel('Time [s]')
ylabel('Temperature [°C]')
legend('Conv.', 'HEV', 'Conv. MM', 'HEV MM', 'Orientation', 'Horizontal')
grid on

% T_atw histogram
figure(); hold on
subplot(211)
histogram(data1.T_atw.val - 273, 40)
ylim([0 500])
title('Histogram of conventional temperatures')
subplot(212)
histogram(data2.T_atw.val - 273, 40)
ylim([0 500])
title('Histogram of HEV temperatures')
xlabel('Temperature [°C]')

% SCR temperatures
figure(); hold on
plot(t1, signals1.T_SCR - 273, 'LineWidth', 2)
plot(t2, signals2.T_SCR - 273, '--', 'LineWidth', 2)
title('SCR temperature')
ylabel('Temperature [°C]')
xlabel('Time [s]')
legend('Conventional', 'HEV', 'Location', 'NorthWest')

% deNOx activity
figure(); hold on
plot(data1.t_vec.val, deNOx_vec_Cu1, 'LineWidth', 2);
plot(data1.t_vec.val, deNOx_vec_Fe1, '--', 'LineWidth', 2)
plot(data2.t_vec.val, deNOx_vec_Cu2, '-.', 'LineWidth', 2)
plot(data2.t_vec.val, deNOx_vec_Fe2, ':', 'LineWidth', 2)
title('deNOx performance for conventional and hybrid powertrains')
xlabel('Time [s]')
ylabel('deNOx [-]')
legend('Conventional CuZ', 'Conventional FeZ', 'Hybrid CuZ', 'Hybrid FeZ', 'Location', 'SouthEast')

% NOx bars
figure()
x_bar = [1, 2; 3, 4];
y_bar = [data1.NOx_tot.val(end), NOx_TP_tot_Cu1, NOx_TP_tot_Fe1; data2.NOx_tot.val(end), NOx_TP_tot_Cu2, NOx_TP_tot_Fe2];
c = categorical({'Conventional', 'Hybrid'});
bar(c,y_bar)
title('Engine out vs. tailpipe NOx for WHTC')
ylabel('NOx [g]')
legend('Engine out', 'CuZ', 'FeZ')
ylim([0 200])
grid on

%------------- END OF CODE --------------









% NAME: plot_fuel_NOx_SS_pareto
%
% PURPOSE: Plotting the 'Pareto' front for a NOx vs. Fuel case study.
%
% OTHER FILES REQUIRED:
%   .m files:
%       createParam__hev_eats_whtc
%
%   .mat files:
%       EATS_HEV_0_1800_K0.2d4_a0_rc10.mat
%       EATS_HEV_0_1800_K0.2d4_a01_rc10.mat
%       EATS_HEV_0_1800_K0.2d4_a02_rc10.mat
%       EATS_HEV_0_1800_K0.2d4_a03_rc10.mat
%       EATS_HEV_0_1800_K0.2d4_a04_rc10.mat
%       EATS_HEV_0_1800_K0.2d4_a05_rc10.mat
%       EATS_HEV_0_1800_K0.2d4_a06_rc10.mat
%       EATS_HEV_0_1800_K0.2d4_a07_rc10.mat
%       EATS_HEV_0_1800_K0.2d4_a08_rc10.mat
%       EATS_HEV_0_1800_K0.2d4_a09_rc10.mat
%       EATS_HEV_0_1800_K0.2d4_a1_rc10.mat
%       ld2_eng_map.mat
%
% SUBFUNCTIONS:
%   none
%
% Author: Hampus Andersson and Fredrik Andersson
% May 2018; Last revision: 29-May-2018

%------------- BEGIN CODE --------------

%% Load data

a0 = load('EATS_HEV_0_1800_K0.2d4_a0_rc10.mat','opt_signals');
a01 = load('EATS_HEV_0_1800_K0.2d4_a01_rc10.mat','opt_signals');
a02 = load('EATS_HEV_0_1800_K0.2d4_a02.mat','opt_signals');
a03 = load('EATS_HEV_0_1800_K0.2d4_a03_rc10.mat','opt_signals');
a04 = load('EATS_HEV_0_1800_K0.2d4_a04_rc10.mat','opt_signals');
a05 = load('EATS_HEV_0_1800_K0.2d4_a05_rc10.mat','opt_signals');
a06 = load('EATS_HEV_0_1800_K0.2d4_a06_rc10.mat','opt_signals');
a07 = load('EATS_HEV_0_1800_K0.2d4_a07_rc10.mat','opt_signals');
a08 = load('EATS_HEV_0_1800_K0.2d4_a08_rc10.mat','opt_signals');
a09 = load('EATS_HEV_0_1800_K0.2d4_a09_rc10.mat','opt_signals');
a1 = load('EATS_HEV_0_1800_K0.2d4_a1_rc10.mat','opt_signals');

%% Pareto construction

F = zeros(1,11);
NOx = F;

F(1) = trapz(a0.opt_signals.t_vec.val, a0.opt_signals.W_f.val);
NOx(1) = trapz(a0.opt_signals.t_vec.val, a0.opt_signals.NOx_TP_Cu.val);

F(2) = trapz(a01.opt_signals.t_vec.val, a01.opt_signals.W_f.val);
NOx(2) = trapz(a01.opt_signals.t_vec.val, a01.opt_signals.NOx_TP_Cu.val);

F(3) = trapz(a02.opt_signals.t_vec.val, a02.opt_signals.W_f.val);
NOx(3) = trapz(a02.opt_signals.t_vec.val, a02.opt_signals.NOx_TP_Cu.val);

F(4) = trapz(a03.opt_signals.t_vec.val, a03.opt_signals.W_f.val);
NOx(4) = trapz(a03.opt_signals.t_vec.val, a03.opt_signals.NOx_TP_Cu.val);

F(5) = trapz(a04.opt_signals.t_vec.val, a04.opt_signals.W_f.val);
NOx(5) = trapz(a04.opt_signals.t_vec.val, a04.opt_signals.NOx_TP_Cu.val);

F(6) = trapz(a05.opt_signals.t_vec.val, a05.opt_signals.W_f.val);
NOx(6) = trapz(a05.opt_signals.t_vec.val, a05.opt_signals.NOx_TP_Cu.val);

F(7) = trapz(a06.opt_signals.t_vec.val, a06.opt_signals.W_f.val);
NOx(7) = trapz(a06.opt_signals.t_vec.val, a06.opt_signals.NOx_TP_Cu.val);

F(8) = trapz(a07.opt_signals.t_vec.val, a07.opt_signals.W_f.val);
NOx(8) = trapz(a07.opt_signals.t_vec.val, a07.opt_signals.NOx_TP_Cu.val);

F(9) = trapz(a08.opt_signals.t_vec.val, a08.opt_signals.W_f.val);
NOx(9) = trapz(a08.opt_signals.t_vec.val, a08.opt_signals.NOx_TP_Cu.val);

F(10) = trapz(a09.opt_signals.t_vec.val, a09.opt_signals.W_f.val);
NOx(10) = trapz(a09.opt_signals.t_vec.val, a09.opt_signals.NOx_TP_Cu.val);

F(11) = trapz(a1.opt_signals.t_vec.val, a1.opt_signals.W_f.val);
NOx(11) = trapz(a1.opt_signals.t_vec.val, a1.opt_signals.NOx_TP_Cu.val);

%% Torque

Mice_F = a1.opt_signals.M_ice.val;
Mem_F  = a1.opt_signals.M_em.val;

Mice_NOx = a0.opt_signals.M_ice.val;
Mem_NOx = a0.opt_signals.M_em.val;

Mice_mid = a05.opt_signals.M_ice.val;
Mem_mid = a05.opt_signals.M_em.val;

%% Energy

load('ld2_eng_map.mat', 'engine_map')

% Only create 'param' if it does not already exist to save time
if ~exist('param', 'var')
    regen_brake = 1400; % Nm
    param = createParam__hev_eats_whtc(engine_map, regen_brake);
end

E = zeros(length(a0.opt_signals.t_vec.val),1);
for i = 2:length(a0.opt_signals.t_vec.val)
    E0(i) = trapz(a0.opt_signals.t_vec.val(1:i), a0.opt_signals.W_f.val(1:i)).*param.ld2.q_HV - ...
 	(a0.opt_signals.x_vec.val(i,7)-a0.opt_signals.x_vec.val(1,7)).* ...
    param.batpack.Q.*param.batpack.Vnom./param.batpack.series_cells;
end
E = zeros(length(a1.opt_signals.t_vec.val),1);
for i = 2:length(a1.opt_signals.t_vec.val)
    E1(i) = trapz(a1.opt_signals.t_vec.val(1:i), a1.opt_signals.W_f.val(1:i)).*param.ld2.q_HV - ...
 	(a1.opt_signals.x_vec.val(i,7)-a1.opt_signals.x_vec.val(1,7)).* ...
    param.batpack.Q.*param.batpack.Vnom./param.batpack.series_cells;
end
figure();
%plot(a0.opt_signals.t_vec.val, E0, a1.opt_signals.t_vec.val, E1)

%% Plotting

figure();
plot(F, NOx, 'o-', 'LineWidth', 2)
title('Fuel consumption vs. NOx emission')
xlabel('Fuel consumption [kg]')
ylabel('NOx emission [g]')

figure();
subplot(2,1,1)
plot(a1.opt_signals.t_vec.val, Mice_F, a1.opt_signals.t_vec.val, Mem_F, 'LineWidth', 2)
title('Torque in fuel optimal case')
ylabel('Torque [Nm]')
legend('ICE','EM')
subplot(2,1,2)
plot(a0.opt_signals.t_vec.val, Mice_NOx, a0.opt_signals.t_vec.val, Mem_NOx, 'LineWidth', 2)
title('Torque in NO_x optimal case')
xlabel('Time [s]')
ylabel('Torque [Nm]')
legend('ICE','EM')

figure();
plot(a0.opt_signals.t_vec.val, E0, a1.opt_signals.t_vec.val, E1, 'LineWidth', 2)
title('Fuel optimal vs. NO_x optimal energy consumption')
xlabel('Time [s]')
ylabel('Energy [J]')
legend('NO_x optimal', 'Fuel optimal', 'Location', 'NorthWest')

%------------- END OF CODE --------------

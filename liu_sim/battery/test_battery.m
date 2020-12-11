clear all
clc
close all

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Create a batterypack from cells %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
load('batcell_2_3Ah.mat');

batcell.NoCells.series      = 197;  % Cells in series
batcell.NoCells.parallel    = 8;   % Cells in parallel
batcell.SOC                 = 0.5;  % Initially charged to 50%
batcell.SOC_max             = 0.5;  % [Desired SOC constraints -
batcell.SOC_min             = 0.25; %  between 0.2-0.8.]

batpack = buildBatteryPack(batcell);
batpack = polyfitBatterySOC(batpack);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Plot SOC curve and voltage vs parallel cells for max power %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

load('em_155kW_3000rpm.mat');

lut_em_eta = em_eta_table(em);
[P_max, pos] = max(em.Tmax .* em.wix);
T_max = em.Tmax(pos);
w_max = em.wix(pos);
eta_max = full(lut_em_eta([w_max T_max]));
P_max_elec = power_em(T_max, w_max, eta_max);

%%
for i = 1:length(em.Tmax)
    eta_max(i) = full(lut_em_eta([em.wix(i) em.Tmax(i)]));
    P_max_elec(i) = power_em(em.Tmax(i), em.wix(i), eta_max(i));
end



parallel_range = 5:30;
plot(5:30,P_max_elec ./ (parallel_range .* batcell.Imax))
title('Cells in parallel to cope with maximum power demand');
ylabel('Battery pack voltage [V]');
xlabel('Battery pack parallel cells [-]')
grid on;
hold on;
plot(batpack.parallel_cells, batpack.Vnom, 'r*')
hold off;
%%
figure(); hold on;
x_range = 0:0.001:1;
k = fill([0.25 0.25 0.5 0.5],[625 675 675 625], 'g','LineStyle', 'none');
alpha(.5);
k.FaceColor = [0 0.6 0];



plot(x_range, interp1(batpack.socix, batpack.Voc, x_range,'pchip'),'LineWidth', 1);
title(['SOC versus U{oc} for ', num2str(batpack.series_cells),'S',num2str(batpack.parallel_cells), 'P battery']) ;
ylabel('U_{oc} [V]');
xlabel('SOC')

% Polynomial fitting area
h = line([0.2 0.2], [625 675], 'color', [1 0 0],'LineWidth', 1);
q = line([0.8 0.8], [625 675], 'color', [1 0 0],'LineWidth', 1);
h.LineStyle = '--';
q.LineStyle = '--';

% Usable SOC area [0.25 0.5]
% y1 = interp1(batpack.socix, batpack.Voc, 0.25,'pchip');
% y2 = interp1(batpack.socix, batpack.Voc, 0.5,'pchip');
% h =line([0.25 0.25], [625 y1+2], 'color', [0 0.7 0],'LineWidth', 2);
% q =line([0.5 0.5], [625 y2+2], 'color', [0 0.7 0],'LineWidth', 2);
% h.LineStyle = ':';
% q.LineStyle = ':';

% Fitted poynom
plot(x_range, SOCtoUoc(x_range,batpack), 'k:', 'LineWidth', 2);

ylim([625 675]);
grid on;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Plot efficiency map for battery %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% I det här läget vet vi vilken spänning batteriet har. Strömmer blir då:
P_em_electric = 0:1000:P_max;   % GIVEN

% Uoc ändras kontinuerligt efter SOC som i sin tur uppdateras genom
% integration av I_b.
U_oc = batpack.Vnom;

U_b = battery_voltage(P_em_electric, U_oc, batpack);

I_b = battery_current(P_em_electric, U_b);

% Effekt som batteriet belastas med. Högre än P_b (effekt som går till
% elmotorn).
P_oc = battery_power(U_oc, I_b);

% eta_b är oviktig men kan vara intressant. Lägre ström, bättre
% effektivitet.
eta_b = battery_efficiency(P_em_electric, P_oc);

% Denna interpolation kan användas för att ge U_oc nytt värde efter
% uppdatering av SOC.
U_oc = interp1(batpack.socix,batpack.Voc, 1, 'pchip');

figure();
contourf(U_b, I_b, repmat(eta_b',1,length(eta_b)));






% NAME: LO_vs_NOx_tradeoff
%
% PURPOSE: Print a 3D figure with the trade-off presented.
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
% May 2018; Last revision: 04-June-2018

%------------- BEGIN CODE --------------

% Data from several runs
Beta = [0, 0.2, 0.5, 0.6, 0.8, 1];
NOx = [18.4422, 16.7245, 14.493, 13.9827, 13.659, 13.4328];
fuel = [1002, 944, 787, 774, 662, 654];
LO  = [652, 681, 710, 741, 773, 793];

% Dense up data
N = 100;
data_grid = linspace(0, 1, N);
NOx2 = interp1(Beta, NOx, data_grid, 'linear');
fuel2= interp1(Beta, fuel, data_grid,'linear');
LO2 = interp1(Beta, LO, data_grid,   'linear');

% Plot
figure(); hold on;
plot3(LO2,NOx2,fuel2,'LineWidth', 2)

plot3(LO2,NOx2,600.*ones(1,N),'g:','LineWidth',1.5);
plot3(LO2,20.*ones(1,N), fuel2,'g:','LineWidth',1.5);
plot3(800.*ones(1,N), NOx2, fuel2,'g:','LineWidth',1.5);

grid on;
title('Fuel, NOx and light-off');
xlabel('Light-off (s)');
ylabel('NOx (g)');
zlabel('Fuel (g)');

%% Plot 2D
figure;
plot(LO,NOx, '-o', 'LineWidth', 1.5);
grid on;
title('NOx vs. Light-off time');
ylabel('NOx (g)');
xlabel('Light-off time (s)');


%------------- END OF CODE --------------

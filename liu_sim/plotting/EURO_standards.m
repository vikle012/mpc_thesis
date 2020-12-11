% NAME: EURO_standards
%
% PURPOSE: Plots the tightening of the EURO standards from 
%          EURO 1 to EURO 6.
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
% June 2018; Last revision: 04-June-2018

%------------- BEGIN CODE --------------

%% CO vs. NOx
figure; hold on;
title('EURO Standards');
xlabel('NOx (g/kWh)');
ylabel('CO (g/kWh)');
ylim([0 5]);
xlim([0 9]);

euro1 = rectangle('Position', [0 0 8 4.5]);
euro1.Curvature = [0.02, 0.02];
euro1.FaceColor = [0.5 0.5 1 1];
euro1.EdgeColor = 'none';

euro2 = rectangle('Position', [0 0 7 4]);
euro2.Curvature = [0.02, 0.02];
euro2.FaceColor = [0.5 1 0.5 0.4];
euro2.EdgeColor = 'none';

euro3 = rectangle('Position', [0 0 5 2.1]);
euro3.Curvature = [0.02, 0.02];
euro3.FaceColor = [1 0.5 0.5 0.4];
euro3.EdgeColor = 'none';

euro4 = rectangle('Position', [0 0 3.5 1.5]);
euro4.Curvature = [0.02, 0.02];
euro4.FaceColor = [1 1 0.5 0.4];
euro4.EdgeColor = 'none';

euro5 = rectangle('Position', [0 0 2 1.5]);
euro5.Curvature = [0.02, 0.02];
euro5.FaceColor = [0.5 1 1 0.4];
euro5.EdgeColor = 'none';

euro6 = rectangle('Position', [0 0 0.4 1.5]);
euro6.Curvature = [0.02, 0.02];
euro6.FaceColor = [1 0 0 0.4];
euro6.EdgeColor = 'none';

%% PM vs. NOx
figure; hold on;
title('EURO Standards');
xlabel('NOx (g/kWh)');
ylabel('PM (g/kWh)');

euro1 = rectangle('Position', [0 0 8 0.36]);
euro1.Curvature = [0.02, 0.02];
euro1.FaceColor = [0 0.6 0 0.4];
euro1.EdgeColor = 'none';

euro2 = rectangle('Position', [0 0 7 0.15]);
euro2.Curvature = [0.02, 0.02];
euro2.FaceColor = [0.7 0.7 0 0.4];
euro2.EdgeColor = 'none';

euro3 = rectangle('Position', [0 0 5 0.1]);
euro3.Curvature = [0.02, 0.02];
euro3.FaceColor = [1 1 0 0.4];
euro3.EdgeColor = 'none';

euro4 = rectangle('Position', [0 0 3.5 0.02]);
euro4.Curvature = [0.02, 0.02];
euro4.FaceColor = [0.7 0.5 0.3 0.5];
euro4.EdgeColor = 'none';

euro5 = rectangle('Position', [0 0 2 0.02]);
euro5.Curvature = [0.02, 0.02];
euro5.FaceColor = [0.7 0.1 0.1 0.4];
euro5.EdgeColor = 'none';

euro6 = rectangle('Position', [0 0 0.4 0.01]);
euro6.Curvature = [0.02, 0.02];
euro6.FaceColor = [1 0 0 1];
euro6.EdgeColor = 'none';

xlabel('NO_x (g/kWh)');
ylabel('PM (g/kWh)');
title('EURO I to EURO VI');
ylim([0 0.4]);
xlim([0 9]);


%------------- END OF CODE --------------



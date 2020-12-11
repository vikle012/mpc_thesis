function lut = em_ploss_table(em)
% FUNCTION: Description of purpose of function
%
% SYNTAX: output1 = em_ploss_table(input1);
%
% INPUTS:
%   input1 - Struct with electric machine data
%
% OUTPUTS:
%   output1 - look-up table for power losses for electric machine
%
% EXAMPLE:
%   [param.em.lut] = em_ploss_table(em);
%
% OTHER FILES REQUIRED:
%   .m files:
%       WHTC_N_table
%       WHTC_dN_table
%   	WHTC_M_table
%
%   .mat files:
%       WHTC_data
%
% SUBFUNCTIONS:
%   none
%
% Author: Hampus Andersson and Fredrik Andersson
% Mars 2018; Last revision: 29-Mars-2018

%------------- BEGIN CODE --------------

tic
disp('Creating look-up-table for em P_loss...');
%% Plotting

% line_grid = 0:1000:24000;
% contourf(em.wix,em.Tix,em.Ploss,line_grid); hold on;
% plot(em.wix,em.Tmax,'r','LineWidth',2);
% plot(em.wix,em.Tmin,'r','LineWidth',2);

%% Creating look-up table
K = 0.25;
wix = linspace(450.*pi./30, 2450.*pi./30, K * length(em.wix) -1);
Tix = linspace(min(em.Tix), max(em.Tix), K * length(em.wix) -1);
%Tix1 = logspace(0, log(max(em.Tix+1))/log(10), K/2 * length(em.wix))-1;
%Tix2 = logspace(0, log(max(em.Tix+1))/log(10), K/2 * length(em.wix))-1;
%Tix = [fliplr(-Tix1), Tix2(2:end)];

% Meshgrid to matrices
[wix_grid, Tix_grid] = meshgrid(wix, Tix);

P_loss = interp2(em.wix, em.Tix, em.Ploss, wix_grid, Tix_grid, 'makita')';
lut = casadi.interpolant('LUT', 'bspline', {wix Tix}, P_loss(:));

fprintf('Finished in %.0f seconds \n \n', toc);

%------------- END OF CODE --------------
end
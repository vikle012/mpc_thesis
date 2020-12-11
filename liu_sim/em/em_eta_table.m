%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Creating look-up-table for electric motor efficiency %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function lut_em = em_eta_table(em)

tic
disp('Creating look-up-table for em_eta...');
em.eta(em.eta < 0.01) = 0.01;
% New index for more precise interpolation to function
K = 5;

wix = linspace(450.*pi./30, 2500.*pi./30, K * length(em.wix) -1);

Tix1 = logspace(0, log(max(em.Tix+1))/log(10), K/2 * length(em.wix))-1;
Tix2 = logspace(0, log(max(em.Tix+1))/log(10), K/2 * length(em.wix))-1;
Tix = [fliplr(-Tix1), Tix2(2:end)];

% Meshgrid to matrices
[wix_grid, Tix_grid] = meshgrid(wix, Tix);
eta = interp2(em.wix, em.Tix, em.eta, wix_grid, Tix_grid, 'linear')';

% Look-up-table creation
lut_em = casadi.interpolant('LUT', 'bspline', {wix Tix}, eta(:));
fprintf('Finished in %.0f seconds \n \n', toc);
end
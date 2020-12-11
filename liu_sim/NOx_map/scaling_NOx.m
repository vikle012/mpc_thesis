%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Testscript for Scaling NOx - OLD FILE -> manual_NO.m is NEW %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

tic
load('EO_NOx_map.mat');

% Variables
N               = 300;
min_NOx         = 50;
scale_factor    = 2.01;

% Redefine struct for SI-units among other
nox.wix = EO_NOx_map.N' .* pi ./30;
nox.Tix = EO_NOx_map.M';
nox.nox = EO_NOx_map.NOx;

% Removes weird NaN in corner of map
nox.nox(1:3,360) = nox.nox(1:3,359);

% Replace all NaN with a random scalar. Will not matter later on
nox.nox(isnan(nox.nox)) = 1500;

%%
% Rescaling wix and Tix to equal length
square_wix      = linspace(min(nox.wix), max(nox.wix), N);
square_Tix      = linspace(min(nox.Tix), max(nox.Tix), N);

% Repmap to matrices
[first_extra_wix_grid, square_Tix_grid] = meshgrid(square_wix, square_Tix);
[old_wix_grid, old_Tix_grid]            = meshgrid(nox.wix, nox.Tix);

% Interpolating new NOx grid for scaled wix and Tix
square_NOx_grid = interp2(old_wix_grid, old_Tix_grid, nox.nox, first_extra_wix_grid, square_Tix_grid, 'linear');

% Creates F function for inter/extra-polating
F = griddedInterpolant(first_extra_wix_grid', square_Tix_grid', square_NOx_grid', 'linear', 'linear');

% Extrapolating from 200 Nm down to 0 Nm and 800 rad/s to 500 rad/s
first_extra_Tix = linspace(0, max(nox.Tix(:,1)), N);
first_extra_wix = linspace(800*pi/30, max(nox.wix(:,1)), N);

% Repmat to mesh
[first_extra_wix_grid, first_extra_Tix_grid] = meshgrid(first_extra_wix, first_extra_Tix);

% Using inter/extra -polant function already compiled
first_extra_NOx = F(first_extra_wix_grid', first_extra_Tix_grid');

% Removes negative NOx values for low torque areas (may not be needed)
first_extra_NOx(first_extra_NOx < min_NOx) = min_NOx;

test.wix = first_extra_wix_grid';
test.Tix = first_extra_Tix_grid' .* scale_factor;
test.nox = first_extra_NOx;


%%%%%%%%%%%%%%%%%%%%%%

line_grid = [0:50:1000, 1000:25:1200];
line_text_size = 8;
axis_text_size = 8;
text_spacing = 550;
xaxis_text = 'Motor speed [rpm]';
yaxis_text = 'Motor torque [Nm]';
title_text = 'NOx emission map';

%LD2_figure();
hold on;
render_nox_map(...
    test, ...
    line_grid, ...
    line_text_size, ...
    axis_text_size, ...
    text_spacing, ...
    xaxis_text, ...
    yaxis_text, ...
    title_text ...
    );

%% 'nearest' extrapolate from an already 'linear' extrapolated mesh

FF = griddedInterpolant(first_extra_wix_grid', first_extra_Tix_grid', first_extra_NOx, 'linear', 'nearest');

second_extra_wix = linspace(500.*pi./30, max(first_extra_wix_grid(1,:)), N)';
second_extra_Tix = first_extra_Tix';

[second_extra_wix_grid, second_extra_Tix_grid] = meshgrid(second_extra_wix, second_extra_Tix);

second_extra_NOx = FF(second_extra_wix_grid', second_extra_Tix_grid');

test.wix = second_extra_wix_grid';
test.Tix = second_extra_Tix_grid';
test.nox = second_extra_NOx;

%%%%%%%%%%%%%%%%%%%%%%

line_grid = [0:50:1000, 1000:25:1200];
line_text_size = 8;
axis_text_size = 8;
text_spacing = 550;
xaxis_text = 'Motor speed [rpm]';
yaxis_text = 'Motor torque [Nm]';
title_text = 'NOx emission map';

%LD2_figure();
hold on;
render_nox_map(...
    test, ...
    line_grid, ...
    line_text_size, ...
    axis_text_size, ...
    text_spacing, ...
    xaxis_text, ...
    yaxis_text, ...
    title_text ...
    );




%%

lut = casadi.interpolant('LUT', 'bspline', {first_extra_wix' (first_extra_Tix .* scale_factor)'}, first_extra_NOx(:));

%%
%% Plot map

toc_last = 0;
% Pick out corresponding values
for j = 1:length(second_extra_wix)
    for i = 1:length(second_extra_Tix)
        nox_model_map.wix(i,j) = second_extra_wix(j);
        nox_model_map.Tix(i,j) = second_extra_Tix(i).*scale_factor;
        nox_model_map.nox(i,j) = full(lut_NO([second_extra_wix(j) second_extra_Tix(i).*scale_factor])); % Converts sparse to full and picks out corresponding element
    end
    if ((toc - toc_last) > 3) % Print progress every 3 second
       fprintf('Loop %d of %d \n', [j N]);
       toc_last = toc;
    end
end
%nox_model_map.nox(nox_model_map.nox > 2000) = nan;
toc


%%
openfig('liu_sim\models\liu_diesel_2\engine_map\liu_diesel_2_engine_map.fig');
line_grid = [0:50:1000, 1000:25:1200];
line_text_size = 8;
axis_text_size = 8;
text_spacing = 550;
xaxis_text = 'Motor speed [rpm]';
yaxis_text = 'Motor torque [Nm]';
title_text = 'NOx emission map';

%LD2_figure();
hold on;
render_nox_map(...
    nox_model_map, ...
    line_grid, ...
    line_text_size, ...
    axis_text_size, ...
    text_spacing, ...
    xaxis_text, ...
    yaxis_text, ...
    title_text ...
    );


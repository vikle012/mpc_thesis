%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Testscript for manual creation of NO-map %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Manual points for interpolation
data = [940 2000 0.95157;
    1120 2000 0.95157;
    %785 1500 0.85639;  % Island
    1110 1500 0.80879;
    %%1000 1700 0.90398;
    600 500 0.66601;
    1420 500 0.33285;
    520 0 0.094874;
    1900 -130 0.04728;    % Lower toward min torque
    1900 1420 0.57082;
    1520 2000 0.66601;
    1730 1500 0.61841;
    1330 1500 0.7612;
    600 800 0.85639;
    1000 800 0.61841;
    1100 -100 0.04728;    % Lower toward min torque
    1000 90 0.094874;
    1800 380 0.23766;
    %%1400 250 0.14247;
    600 120 0.19006;
    1000 420 0.33285;
    %%1000 2350 0.95157;
    1330 2350 0.95157;
    830 1400 0.85639;
    %920 1400 0.85639;  % Island
    800 1100 0.80879;
    870 1850 0.90398;
    1800 1250 0.57082;
    %%1000 1550 0.85639;
    2400 -140 0.04728;   % Special % Lower toward min torque
    1600 1950 0.61841;
    1500 1750 0.7612;
    1400 2000 0.80879;
    1000 2100 0.999;
    %%700 1300 0.904;
    %%750 1500 0.9;
    %%850 1750 0.9       % Island
    1400 -130 0.04728;   % Lower toward min torque
    800 -50 0.04727;     % Lower toward min torque
    1200 85 0.1;         
    1400 75 0.1;
    1400 940 0.52322;
    1200 300 0.19
    ];

% Extract x,y,z from data
x = data(:,1)';
y = data(:,2)';
z = data(:,3)';

% Inter- and extrapolating according to new grid
 
zi = griddata(x,y,z,xi,yi, 'v4').*2000;

% For plotting
test.wix = xi ./ 30 .* pi;
test.Tix = yi;
test.nox = zi;

line_grid = [0:100:2000];
line_text_size = 8;
axis_text_size = 8;
text_spacing = 550;
xaxis_text = 'Motor speed [rpm]';
yaxis_text = 'Motor torque [Nm]';
title_text = 'manual NOx emission map';

clf
hold on;
%LD2_figure();
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
plot(x,y,'k*')

% Compare with efficiency lines for ICE engine
load('ld2_eng_map.mat');
contour(engine_map.rpm_grid, engine_map.torque_grid, engine_map.efficiency_grid,'k','ShowText','on', 'LineWidth', 2)
plot(engine_map.rpm_range, engine_map.torque_grid(end,:), 'r-', 'LineWidth', 2)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Testscript for manual creation of NO2/NO-map %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Manual points for interpolation
data = [500 0 9;
    800 70 9;
    %1000 130 9;
    %1200 150 9;
    2400 200 9;
    %600 750 3;
    1000 1075 3;
    1200 1425 3;
    1350 2000 3;
    1500 2075 3;
    1300 1750 3;
    600 1500 3;     % Special
    850 900 3;
    500 700 4;
    600 450 4;
    720 500 4;
    1000 550 4;
    1200 640 4;
    1400 920 4;
    1600 1190 4;
    1900 1370 4;
    2400 1700 4;    % Special
    500 260 6;
    600 260 6;
    1000 360 6;
    1400 365 6;
    2000 365 6;     % Special
    2400 370 6;     % Special
    600 0 11;
    800 0 11;
    1000 0 11;
    1200 0 11;
    1400 0 11;
    1600 20 11;
    1900 50 11;
    2400 70 11;
    500 -70 16;
    600 -60 16;
    800 -65 16;
    1000 -70 16;
    1200 -100 16;
    1400 -250 16;    % Special
    800 1360 3;
    880 1500 3;
    950 2000 3;
    1000 2200 3;
    1700 2400 3;
    600 350 5;
    800 400 5;
    1000 450 5;
    1200 450 5;
    1400 450 5;
    1600 550 5;
    2400 1000 5;    % Special
    2300 950 5;
    2200 900 5;     % Special
    2100 850 5;
    2000 800 5;     % Special
    500 350 5;
    1800 680 5;
    1900 750 5;
    600 75 8;
    500 75 8;
    800 150 8;
    1000 175 8;
    1200 200 8;
    1400 225 8;
    1600 250 8;
    2000 250 8;
    2200 250 8;
    2400 250 8;
    2200 650 5.5;       % Special
    %500 2400 3;         % Special
    %700 850 3;
    500 1000 3;
    655 800 3;          % Special
    ];

% Extract x,y,z from data
x = data(:,1)';
y = data(:,2)';
z = data(:,3)';

% Inter- and extrapolating according to new grid
[xi,yi] = meshgrid(500:5:2400, -250:5:2400);
zi = griddata(x,y,z,xi,yi, 'v4');

% For plotting
test.wix = xi ./ 30 .* pi;
test.Tix = yi;
test.nox = zi;

line_grid = [0:1:21];
line_text_size = 8;
axis_text_size = 8;
text_spacing = 550;
xaxis_text = 'Motor speed [rpm]';
yaxis_text = 'Motor torque [Nm]';
title_text = 'manual NO2/NO emission map';


clf
hold on;
%LD2_figure();
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
plot(x,y,'k*')

% Compare with efficiency lines for ICE engine
load('ld2_eng_map.mat');
contour(engine_map.rpm_grid, engine_map.torque_grid, engine_map.efficiency_grid,'k','ShowText','on', 'LineWidth', 2)
plot(engine_map.rpm_range, engine_map.torque_grid(end,:), 'r-', 'LineWidth', 2)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Test look-up-table created in 'emission_lut_XX' %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

w_doodle = 450:10:2450;
T_doodle = -250:10:2400;

lut_NO = param.ld2.lut_NO;
lut_NO2NO = param.ld2.lut_NO2NO;

tic
toc_last = 0;
% Pick out corresponding values
for j = 1:length(w_doodle)
    for i = 1:length(T_doodle)
        no_model_map.wix(i,j) = w_doodle(j) .*pi ./30;
        no_model_map.Tix(i,j) = T_doodle(i);
        no_model_map.nox(i,j) = full(lut_NO([w_doodle(j) T_doodle(i)])); % Converts sparse to full and picks out corresponding element
        no2_model_map.wix(i,j) = w_doodle(j) .*pi ./30;
        no2_model_map.Tix(i,j) = T_doodle(i);
        no2_model_map.nox(i,j) = no_model_map.nox(i,j)*full(lut_NO2NO([w_doodle(j) T_doodle(i)]));
    end
    if ((toc - toc_last) > 3) % Print progress every 3 second
       fprintf('Loop %d of %d \n', [j length(w_doodle)]);
       toc_last = toc;
    end
end
%nox_model_map.nox(nox_model_map.nox > 2000) = nan;
toc


%openfig('liu_sim\models\liu_diesel_2\engine_map\liu_diesel_2_engine_map.fig');
line_grid_no = [0:100:2200]; % NO
line_grid_no2 = [0:5:100]; % NO2NO
line_text_size = 8;
axis_text_size = 8;
text_spacing = 550;
xaxis_text = 'Motor speed [rpm]';
yaxis_text = 'Motor torque [Nm]';
title_text_no = 'NO emission map';
title_text_no2 = 'NO_2 emission map';

LD2_figure(); % Plot NO map
hold on;
render_nox_map(...
    no_model_map, ...
    line_grid_no, ...
    line_text_size, ...
    axis_text_size, ...
    text_spacing, ...
    xaxis_text, ...
    yaxis_text, ...
    title_text_no ...
    );
load('ld2_eng_map.mat');
contour(engine_map.rpm_grid, engine_map.torque_grid, engine_map.efficiency_grid,'k','ShowText','on', 'LineWidth', 2)
plot(engine_map.rpm_range, engine_map.torque_grid(end,:), 'r-', 'LineWidth', 2)
plot(engine_map.rpm_range, (-3.356001893629823e-05.*engine_map.rpm_range.^2 + 0.002507880152583.*engine_map.rpm_range -76.058728963433370), 'r-', 'LineWidth', 2)


LD2_figure(); % Plot NO2 map
hold on;
render_nox_map(...
    no2_model_map, ...
    line_grid_no2, ...
    line_text_size, ...
    axis_text_size, ...
    text_spacing, ...
    xaxis_text, ...
    yaxis_text, ...
    title_text_no2 ...
    );
load('ld2_eng_map.mat');
contour(engine_map.rpm_grid, engine_map.torque_grid, engine_map.efficiency_grid,'k','ShowText','on', 'LineWidth', 2)
plot(engine_map.rpm_range, engine_map.torque_grid(end,:), 'r-', 'LineWidth', 2)
plot(engine_map.rpm_range, (-3.356001893629823e-05.*engine_map.rpm_range.^2 + 0.002507880152583.*engine_map.rpm_range -76.058728963433370), 'r-', 'LineWidth', 2)





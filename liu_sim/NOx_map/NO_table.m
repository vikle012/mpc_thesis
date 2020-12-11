% --------------------------------------
% -- Returns look-up-table for NO map --
% --------------------------------------
function [lut] = NO_table(max_NO_ppm)
tic
disp('Creating look-up-table for NO...');

% Manually inserted dataset for interpolation
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

% Extract w (rpm), M (Nm) and NO (ppm) from data
w   = data(:,1)';
M   = data(:,2)';
NO  = data(:,3)'; % Scaling

% Define grid for scattered interpolation
[wi, Mi] = meshgrid(450:5:2450, -200:5:2400);

% Interpolate over grid using 'v4' method. Other methods does not
% extrapolate the entire grid
NOi = (griddata(w, M, NO, wi, Mi, 'v4') .* max_NO_ppm)';

% Create look-up-table for the interpolated NO-map
lut = casadi.interpolant('LUT', 'bspline', {wi(1,:) Mi(:,1)}, NOi(:));
fprintf('Finished in %.0f seconds \n \n', toc);
end


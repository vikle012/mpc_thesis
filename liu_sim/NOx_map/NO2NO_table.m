% --------------------------------------
% -- Returns look-up-table for NO map --
% --------------------------------------
function [lut] = NO2NO_table()
tic
disp('Creating look-up-table for NO2 / NO...');
scaling = 1./100;

% Manually inserted dataset for interpolation
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

% Extract w (rpm), M (Nm) and NO2/NO (%) from data
w   = data(:,1)';
M   = data(:,2)';
NO  = data(:,3)'; % Scaling

% Define grid for scattered interpolation
[wi, Mi] = meshgrid(450:5:2450, -200:5:2400);

% Interpolate over grid using 'v4' method. Other methods does not
% extrapolate the entire grid
NOi = (griddata(w, M, NO, wi, Mi, 'v4') .* scaling)';

% Create look-up-table for the interpolated NO-map
lut = casadi.interpolant('LUT', 'bspline', {wi(1,:) Mi(:,1)}, NOi(:));
fprintf('Finished in %.0f seconds \n \n', toc);

end


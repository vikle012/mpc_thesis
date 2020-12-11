% ---------------------------------------------
% -- Returns look-up-table for NOx map [g/s] --
% ---------------------------------------------
function [lut] = NOx_table
tic
disp('Creating look-up-table for NOx...');

load 'NOx_map.mat'
load 'ld2_eng_map.mat'

M = NOx_map.M;
N = NOx_map.N;
NOx = NOx_map.W_NOx;
%Exh = NOx_map.W_exh;

% Define minimum torque and interpolate on suitable speed grid
M_min = (-3.356001893629823e-05 .* engine_map.rpm_range .^ 2 ...
    + 0.002507880152583 .* engine_map.rpm_range ...
    - 76.058728963433370);
M_min_13 = interp1(engine_map.rpm_range, M_min, [500; 650; 800; 950; 1100; 1250; 1400; 1550; 1700; 1850; 2000; 2150; 2400]);

%% Data treatment
% Rearrange measurement vectors (170x1) into matrices (13x13), last element
% lost.
N_mat = zeros(13,13);
M_mat = zeros(13,13);
NOx_mat = zeros(13,13);
for i=1:12
    N_mat(1:13,1) = N(1:13);
    N_mat(1:13,i+1) = N(1+i*13:13+i*13);
    
    M_mat(1:13,1) = M(1:13);
    M_mat(1:13,i+1) = M(1+i*13:13+i*13);
    
    NOx_mat(1:13,1) = NOx(1:13);
    NOx_mat(1:13,i+1) = abs(NOx(1+i*13:13+i*13));
    
    %Exh_mat(1:13,1) = Exh(1:13);
    %Exh_mat(1:13,i+1) = Exh(1+i*13:13+i*13);
end

N_mat   = N_mat.*60; % RPS to RPM

%%% Added points (optional)
 NOx_mat = [NOx_mat, ones(13,1).*0];
 N_mat   = [N_mat,   [500; 650; 800; 950; 1100; 1250; 1400; 1550; 1700; 1850; 2000; 2150; 2400]];
 M_mat   = [M_mat,   M_min_13];

%%% Altered points (for smoothening)
N_mat(3,1) = 1900;
N_mat(2,1) = 2000;
M_mat(2,1) = 2000;
M_mat(2,2) = 1950;
N_mat(2,2) = 1850;
M_mat(1,1) = 1850;
N_mat(1,1) = 2100;
M_mat(1,2) = 1760;
N_mat(4,1) = 1525;
M_mat(4,1) = 2300;
M_mat(5,1) = 2337;
M_mat(1,3) = 1550;
N_mat(1,2) = 2050;

%% Interpolation
NOx_mat = NOx_mat.*0.4/0.0011; % Max NOx from 1800s optimization used as scaling factor

[N2,M2] = meshgrid(500:10:2400, -300:10:2400);
NOx_grid = (griddata(N_mat, M_mat, NOx_mat, N2, M2, 'v4'))';
NOx_grid(NOx_grid < 0) = 0;
lut = casadi.interpolant('LUT', 'bspline', {N2(1,:) M2(:,1)'}, NOx_grid(:)');
fprintf('Finished in %.0f seconds \n \n', toc);


%% Plot
%Data
%Min/max torque curves
% M_max = engine_map.torque_grid(end,:);
% M_grid = linspace(500, 2400, 50);
% 
% contourf(N2, M2, interpalerp',[0:0.01:0.4])
% hold on
% plot(M_grid, M_max, 'r', 'LineWidth', 2)
% hold on
% plot(M_grid, M_min, 'r', 'LineWidth', 2)
% plot(N_mat, M_mat, 'bx') % Data points
% plot(N_mat, M_mat, 'bx', N_mat(:,15), M_mat(:,15), 'rx') % Added points

end







%%
N_range = engine_map.rpm_range;
M_range = engine_map.torque_grid(end,:);
plot(N_range,M_range)
%% SECTION 1
close all;
figure(1); hold on;
plot(N_range(1:10), M_range(1:10), 'ro');
N1 = linspace(N_range(1), N_range(10), 20);
M1 = interp1(N_range(1:10), M_range(1:10), N1, 'pchip');

plot(N1,M1,'b','LineWidth', 1.5);
c1 = polyfit(N1, M1, 3);
plot(N1, polyval(c1, N1), 'k');

%% SECTION 2 (( overkill as it is 2396 ))
figure(1); hold on;
plot(N_range(10:21), M_range(10:21), 'ro');
N2 = linspace(N_range(10), N_range(21), 20);
M2 = interp1(N_range(10:21), M_range(10:21), N2, 'pchip');

plot(N2, M2, 'b', 'LineWidth', 1.5);
c2 = polyfit(N2, M2, 1);
plot(N2, polyval(c2, N2), 'k');
%% SECTION 3
figure(1); hold on;
plot(N_range(21:37), M_range(21:37), 'ro');
N3 = linspace(N_range(21), N_range(37), 20);
M3 = interp1(N_range(21:37), M_range(21:37), N3, 'pchip');

plot(N3, M3, 'b', 'LineWidth', 1.5);
c3 = polyfit(N3, M3, 2);
plot(N3, polyval(c3, N3), 'k');
%% SECTION 4
figure(1); hold on;
plot(N_range(37:end), M_range(37:end), 'ro');
N4 = linspace(N_range(37), N_range(end), 20);
M4 = interp1(N_range(37:end), M_range(37:end), N4, 'pchip');

plot(N4, M4, 'b', 'LineWidth', 1.5);
c4 = polyfit(N4, M4, 2);
plot(N4, polyval(c4, N4), 'k');

%%



torque_max = @(N,M) M - [(2.350053575605094e-05.*N.^3 - 0.037356370305584.*N.^2 + ...
    20.861397967062427.*N - 2.731879241606560e+03); ...
    2396; ...
    (6.128830375899390e-04.*N.^2 + 3.100167284419236.*N + 5.373603733118146e+03); ...
    (7.715040623355741e-04.*N.^2 - 5.018158616034680.*N + 8.440680865836488e+03) ...
    ];


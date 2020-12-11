%% Option 1
% Fits a polynomial to the maximum and minimum power curves from measured data.
% The polynomials are implemented in M_em_max_fun, used as a constraint in
% electric_motor.m
% Yields 10th order polynomials. Could reasonably be replaced with 1 linear
% and 1 lower order polynomial.
clc
load em_155kW_3000rpm.mat

% Load measured data
M_em_max = em.Tmax;
M_em_min = em.Tmin;
N_em = em.wix*30/pi;

% Fit a polynomial and define explicitly
Pc_max = polyfit(N_em,M_em_max,10);
P_max = Pc_max(1).*N_em.^10 + ...
    Pc_max(2).*N_em.^9 + ...
    Pc_max(3).*N_em.^8 + ...
    Pc_max(4).*N_em.^7 + ...
    Pc_max(5).*N_em.^6 + ...
    Pc_max(6).*N_em.^5 + ...
    Pc_max(7).*N_em.^4 + ...
    Pc_max(8).*N_em.^3 + ...
    Pc_max(9).*N_em.^2 + ...
    Pc_max(10).*N_em + ...
    Pc_max(11);
% Plot fitted polynomial vs. measured M_em_max
plot(N_em,M_em_max,N_em,P_max)
hold on

% Fit a polynomial and define explicitly
Pc_min = polyfit(N_em,M_em_min,10);
P_min = Pc_min(1).*N_em.^10 + ...
    Pc_min(2).*N_em.^9 + ...
    Pc_min(3).*N_em.^8 + ...
    Pc_min(4).*N_em.^7 + ...
    Pc_min(5).*N_em.^6 + ...
    Pc_min(6).*N_em.^5 + ...
    Pc_min(7).*N_em.^4 + ...
    Pc_min(8).*N_em.^3 + ...
    Pc_min(9).*N_em.^2 + ...
    Pc_min(10).*N_em + ...
    Pc_min(11);
% Plot fitted polynomial vs. measured M_em_min
plot(N_em,M_em_min,N_em,P_min)
%% Option 2
% Fits a straight line + 3rd order polynomial instead of 10th order polynomial
clc
load em_155kW_3000rpm.mat
% Max and min coefficients
Pc_max1 = polyfit(N_em(1:22),M_em_max(1:22),1);
Pc_max2 = polyfit(N_em(23:end),M_em_max(23:end),3);

Pc_min1 = polyfit(N_em(1:22),M_em_min(1:22),1);
Pc_min2 = polyfit(N_em(23:end),M_em_min(23:end),3);

% Evaluate polynomials
%P_max1 = Pc_max1(1).*N_em + Pc_max1(2);
%P_max2 = Pc_max2(1).*N_em.^2 + Pc_max2(2).*N_em + Pc_max2(3);
P_max2 = polyval(Pc_max1,N_em);
P_max2 = polyval(Pc_max2,N_em);

P_min1 = polyval(Pc_min1,N_em);
P_min2 = polyval(Pc_min2,N_em);

% Plot stuff
plot(N_em,M_em_max,'r--','LineWidth',2)
hold on
plot(N_em,P_max1,'b',N_em,P_max2,'b')
plot(N_em,M_em_min,'r--','LineWidth',2)
plot(N_em,P_min1,'b',N_em,P_min2,'b')
legend('Maximum torque','Modeled constraints')
title('Electric motor maximum torque constraints')
ylabel('Torque [Nm]')
xlabel('Speed [rpm]')
axis([0 3000 -2500 2500])
function [deNOx_Cu, deNOx_Fe, NOx_TP_Cu, NOx_TP_Fe] = deNOx_fun(T_aSCR, signals)
% FUNCTION: Fetch deNOx performance percentage based on SCR temperature.
% Performance level y = rough estimation based on article data.
% To plot deNOx(T) curves, uncomment the temperature vector below and run
% the function by ctrl+enter.
%
% I LIKE CELSIUS
%
% SYNTAX: [output1, output2] = deNOx_fun(input1, input2, input3);
%
% INPUTS:
%   input1 - EATS signals
%   input2 - light-off temperature
%   input3 - ld2 signals
%
% OUTPUTS:
%   output1 - deNOx performance
%   output2 - absolute NOx
%
% EXAMPLE:
%   [deNOx, NOx_TP] = deNOx_fun(T_aSCR, 200, signals);
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
% April 2018; Last revision: 17-April-2018

%------------- BEGIN CODE --------------


%T = 0:1:700; % Makes function "executable". Use when plotting deNOx curves

T = T_aSCR - 273; % To celcius. Use when running full system.

%% Create temp/denox line - "Cu-behavior"
% Zero line
y_1 = zeros(size(T));

% Increase line
k_2 = 0.0115;%1/10;
m_2 = -k_2.*110;%(i);
y_2 = k_2*T+m_2;

% Maximum line
y_3 = ones(size(T));%.98*ones(size(T));

% Decrease line
k_4 = -0.0016;%%-1/100;
m_4 = 0.98 -k_4*350;
y_4 = k_4*T+m_4;

%% Create temp/denox line - "Fe-behavior"
% Zero line
y_5 = zeros(size(T));

% Increase line
k_6 = 0.0046;
m_6 = -k_6.*110;%(i);
y_6 = k_6*T+m_6;

% Maximum line
k_7 = 6.6667e-04;
m_7 = 0.82 - 280*k_7;
y_7 = k_7*T+m_7;%.98*ones(size(T));

% Decrease line
k_8 = -5.0000e-04;%-1/100;
m_8 = 0.95 - 475*k_8;
y_8 = k_8*T+m_8;

%% Continiously differentiable functions
% |x| = sqrt(x^2) ~ sqrt(x^2 + e), e > 0 
cont_abs = @(x, e) sqrt(x.^2 + e);
% max(a, b) = ( a + b + |a - b|)/2;
cont_max = @(x1, x2, e) (x1 + x2 + cont_abs(x1-x2, e))/2;
% min(a, b) = ( a + b - |a - b|)/2;
cont_min = @(x1, x2, e) (x1 + x2 - cont_abs(x1-x2, e))/2;

% Compile lines using continious min/max
deNOx_Cu = cont_min(y_2, y_3, .008);
deNOx_Cu = cont_min(deNOx_Cu, y_4, .01);
deNOx_Cu = cont_max(deNOx_Cu,y_1,.001);
y_Cu = deNOx_Cu;

deNOx_Fe = cont_min(y_6, y_7, .008);
deNOx_Fe = cont_min(deNOx_Fe, y_8, .01);
deNOx_Fe = cont_max(deNOx_Fe,y_5,.001);
y_Fe = deNOx_Fe;
%% Calculate tailpipe NOx
NOx_TP_Cu = signals.NOx_EO.*(1-deNOx_Cu);
NOx_TP_Fe = signals.NOx_EO.*(1-deNOx_Fe);
%% Plot deNOx curve
% figure(1);%clf;
% subplot(211);hold on
% plot(T,y_1,'--k')
% plot(T,y_2,'--k')
% plot(T,y_3,'--k')
% plot(T,y_4,'--k')
% 
% plot(T,y_Cu, 'LineWidth', 2)
% 
% ylim([-1 2])
% %xlabel('T [ °C]');
% ylabel('deNOx [-]');
% title('deNOx performance for CuZ catalyst')
% %figure(2);%clf;
% subplot(212);hold on
% plot(T,y_5,'--k')
% plot(T,y_6,'--k')
% plot(T,y_7,'--k')
% plot(T,y_8,'--k')
% 
% plot(T,y_Fe, 'LineWidth', 2)
% 
% ylim([-1 2])
% xlabel('T [°C]');
% ylabel('deNOx [-]');
% title('deNOx performance for FeZ catalyst')
% 
% figure(3)
% plot(T, y_Cu, T, y_Fe)
% title('deNOx performance for CuZ and FeZ catalysts')
% legend('CuZ', 'FeZ','location', 'SouthEast')
% grid on
% xlabel('T [°C]');
% ylabel('deNOx [-]');
%end

%------------- END OF CODE --------------
end




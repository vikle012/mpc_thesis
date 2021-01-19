function [n_lo, n_pref, n_hi] = WHTC_engine_speed_values(engine_map)
% FUNCTION: Finding rpm values for denormalization of WHTC engine speed.
%
% SYNTAX: [output1, output2, output3] = WHTC_engine_speed_values(input1);
%
% INPUTS:
%   input1 - Struct with the engine map
%
% OUTPUTS:
%   output1 - n_lo is the lowest speed where the power is 55 per cent of 
%             maximum power
%   output2 - n_pref is the engine speed where the max. torque integral 
%             is 51 per cent of the whole integral 
%   output3 - n_hi is the highest speed where the power is 70 per cent 
%             of maximum power
%
% EXAMPLE:
%   [n_lo, n_pref, n_hi] = WHTC_engine_speed_values(engine_map);
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
% Mars 2018; Last revision: 28-Mars-2018

%------------- BEGIN CODE --------------

% figure(1); hold on;
% title('Scaling for WHTC');
% ylabel('Maximum power (W)');
% xlabel('Engine speed (rpm)');

N = engine_map.rpm_range.*pi./30;
M = engine_map.torque_grid(end,:);
P = N.*M;
P_max = max(P);

% plot(N.*30./pi,P, 'g', 'LineWidth', 2);

%% n_lo
section1 = 1:10;
c1 = polyfit(N(section1), P(section1), 3);

eqn = @(x) polyval(c1,x) - 0.55*P_max;
n_lo = fsolve(eqn, 0, optimoptions('fsolve','display', 'off')) *30/pi;

% plot(N(section1).*30./pi, polyval(c1,N(section1)), 'r--', 'LineWidth', 1.5);
% plot(n_lo, polyval(c1, n_lo/30*pi), 'ko');
% txt1 = '  \leftarrow n_{lo} = 815 rpm';
% text(n_lo, polyval(c1, n_lo/30*pi),txt1, 'HorizontalAlignment','left')

%% n_hi
section4 = 37:50;
c4 = polyfit(N(section4),P(section4), 1);
eqn = @(x) polyval(c4,x) - 0.7*P_max;
n_hi = fsolve(eqn, 0, optimoptions('fsolve','display', 'off')) *30/pi;

% plot(N(section4).*30./pi, polyval(c4,N(section4)), 'r--', 'LineWidth', 1.5);
% plot(n_hi, polyval(c4, n_hi*pi/30), 'ko');
% txt1 = 'n_{hi} = 2301 rpm \rightarrow  ';
% text(n_hi, polyval(c4, n_hi/30*pi),txt1, 'HorizontalAlignment','right');

%% n_95h
eqn = @(x) polyval(c4,x) - 0.95*P_max;
n_95h = fsolve(eqn, 0, optimoptions('fsolve','display', 'off')) *30/pi;

% plot(N(section4).*30./pi, polyval(c4,N(section4)), 'r--', 'LineWidth', 1.5);
% plot(n_95h, polyval(c4, n_95h*pi/30), 'ko');
% txt1 = ' \leftarrow n_{95h} = 1966 rpm';
% text(n_95h, polyval(c4, n_95h/30*pi),txt1, 'HorizontalAlignment','left');

%% ANDRA
section2 = 10:21;
c2 = polyfit(N(section2), P(section2), 3);
% plot(N(section2).*30./pi, polyval(c2,N(section2)), 'b--', 'LineWidth', 1.5);

%% TREDJE
section3 = 22:36;
c3 = polyfit(N(section3), P(section3), 3);
% plot(N(section3).*30./pi, polyval(c3,N(section3)), 'b--', 'LineWidth', 1.5);

%%

%% n_pref
int_max = trapz(polyval(c1,N(1:10))) + ...
    trapz(polyval(c2,N(10:22))) + ...
    trapz(polyval(c3,N(22:37))) + ...
    trapz(polyval(c4,N(37:39)));

int_pref = 0.51*int_max;
int24 = trapz(polyval(c1,N(1:10))) + ...
    trapz(polyval(c2,N(10:22))) + ...
    trapz(polyval(c3,N(22:24)));

eqn = @(x) int24*x - int_pref;
find_x = fsolve(eqn, 0, optimoptions('fsolve','display', 'off'));
n_pref = find_x * N(24)*30/pi;

% plot(n_pref, polyval(c3, n_pref*pi/30), 'ko');
% txt1 = 'n_{pref} = 1418 rpm \rightarrow      ';
% text(n_pref, polyval(c3, n_pref/30*pi),txt1, 'HorizontalAlignment','right');
% 
% [~,temp] = min(abs(n_pref-N.*30./pi));
% line([n_pref, n_pref],[0 P(temp)]);
% line([500 500],[0 P(1)]);
% [~,temp] = min(abs(n_95h-N.*30./pi));
% line([1966 1966],[0 P(temp)]);

%%
% fprintf('\nn_lo   = %1.0f  rpm\n', n_lo);
% fprintf('n_hi   = %1.0f rpm\n', n_hi);
% fprintf('n_95h  = %1.0f rpm\n', n_95h);
% fprintf('n_pref = %1.0f rpm\n', n_pref);
% fprintf('n_idle = %3.0f  rpm\n', 500);

%hold off;

%------------- END OF CODE --------------
end
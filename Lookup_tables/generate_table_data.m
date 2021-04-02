%% Load data

load('parameterData')
load('table_data_relaxed')

%% Use find_reference, find_reference_relax and find_reference_relax2

% max_torque_curve
% 
% x1_table_data = nan*ones(21, length(N_e));
% x2_table_data = nan*ones(21, length(N_e));
% x3_table_data = nan*ones(21, length(N_e));
% x4_table_data = nan*ones(21, length(N_e));
% x5_table_data = nan*ones(21, length(N_e));
% u1_table_data = nan*ones(21, length(N_e));
% u2_table_data = nan*ones(21, length(N_e));
% u3_table_data = nan*ones(21, length(N_e));
% 
% at = [];
% count = 0;
% 
% for k = 1:length(N_e)
%    
%     Me_max = M_e(k);
%     at_ne = N_e(k);
%     
%     j = 0;
%     for i = 0:5:100
%         Me = i/100*Me_max;
%         
%         j = j + 1;
%         
%         [x_ref, u_ref, flag] = find_reference(Me, at_ne, model);
%         
%         if flag == 1
%             x1_table_data(j, k) = x_ref(1);
%             x2_table_data(j, k) = x_ref(2);
%             x3_table_data(j, k) = x_ref(3);
%             x4_table_data(j, k) = x_ref(4);
%             x5_table_data(j, k) = x_ref(5);
%             u1_table_data(j, k) = u_ref(1);
%             u2_table_data(j, k) = u_ref(2);
%             u3_table_data(j, k) = u_ref(3);
%         else
%             [x_ref, u_ref, flag] = find_reference_relax(Me, at_ne, model);
%             
%              if flag == 1
%                 x1_table_data(j, k) = x_ref(1);
%                 x2_table_data(j, k) = x_ref(2);
%                 x3_table_data(j, k) = x_ref(3);
%                 x4_table_data(j, k) = x_ref(4);
%                 x5_table_data(j, k) = x_ref(5);
%                 u1_table_data(j, k) = u_ref(1);
%                 u2_table_data(j, k) = u_ref(2);
%                 u3_table_data(j, k) = u_ref(3);
%              else
%                 [x_ref, u_ref] = find_reference_relax2(Me, at_ne, model);
%                 
%                 x1_table_data(j, k) = x_ref(1);
%                 x2_table_data(j, k) = x_ref(2);
%                 x3_table_data(j, k) = x_ref(3);
%                 x4_table_data(j, k) = x_ref(4);
%                 x5_table_data(j, k) = x_ref(5);
%                 u1_table_data(j, k) = u_ref(1);
%                 u2_table_data(j, k) = u_ref(2);
%                 u3_table_data(j, k) = u_ref(3);
%                 
%             end
%         end
%     end
% end

%% Plot

load('table_data_relaxed')

M_e_axis = 0:5:100;
n_e_axis = 500:25:2000;

h1 = figure(1);
grid on
surf(n_e_axis, M_e_axis, x1_table_data)
xlabel('Engine speed [rpm]')
ylabel('Normalized torque [%]')
zlabel('p_{im} [Pa]')
colorbar
% set(h1,'Units','Inches');
% pos = get(h1,'Position');
% set(h1,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
% print(h1,'Figures/table_data_x1','-dpdf','-r0')


h2 = figure(2);
surf(n_e_axis, M_e_axis, x2_table_data)
xlabel('Engine speed [rpm]')
ylabel('Normalized torque [%]')
zlabel('p_{em} [Pa]')
colorbar
% set(h2,'Units','Inches');
% pos = get(h2,'Position');
% set(h2,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
% print(h2,'Figures/table_data_x2','-dpdf','-r0')

h3 = figure(3);
surf(n_e_axis, M_e_axis, x3_table_data)
xlabel('Engine speed [rpm]')
ylabel('Normalized torque [%]')
zlabel('X_{Oim} [-]')
colorbar
% set(h3,'Units','Inches');
% pos = get(h3,'Position');
% set(h3,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
% print(h3,'Figures/table_data_x3','-dpdf','-r0')

h4 = figure(4);
surf(n_e_axis, M_e_axis, x4_table_data);
xlabel('Engine speed [rpm]')
ylabel('Normalized torque [%]')
zlabel('X_{Oem} [-]')
colorbar
% set(h4,'Units','Inches');
% pos = get(h4,'Position');
% set(h4,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
% print(h4,'Figures/table_data_x4','-dpdf','-r0')

h5 = figure(5);
surf(n_e_axis, M_e_axis, x5_table_data)
xlabel('Engine speed [rpm]')
ylabel('Normalized torque [%]')
zlabel('\omega_{t} [rad/s]')
colorbar
% set(h5,'Units','Inches');
% pos = get(h5,'Position');
% set(h5,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
% print(h5,'Figures/table_data_x5','-dpdf','-r0')

h6 = figure(6);
surf(n_e_axis, M_e_axis, u1_table_data)
xlabel('Engine speed [rpm]')
ylabel('Normalized torque [%]')
zlabel('u_\delta [mg/cycle]')
colorbar
% set(h6,'Units','Inches');
% pos = get(h6,'Position');
% set(h6,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
% print(h6,'Figures/table_data_u1','-dpdf','-r0')

h7 = figure(7);
surf(n_e_axis, M_e_axis, u2_table_data)
xlabel('Engine speed [rpm]')
ylabel('Normalized torque [%]')
zlabel('u_{egr} [%]')
colorbar
% set(h7,'Units','Inches');
% pos = get(h7,'Position');
% set(h7,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
% print(h7,'Figures/table_data_u2','-dpdf','-r0')

h8 = figure(8);
surf(n_e_axis, M_e_axis, u3_table_data)
xlabel('Engine speed [rpm]')
ylabel('Normalized torque [%]')
zlabel('u_{vgt} [%]')
colorbar
% set(h8,'Units','Inches');
% pos = get(h8,'Position');
% set(h8,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
% print(h8,'Figures/table_data_u3','-dpdf','-r0')


%% More plots

[m,n] = size(x1_table_data);
M_e_data = zeros(m,n);
x1_dot_data = zeros(m,n);
x2_dot_data = zeros(m,n);
x3_dot_data = zeros(m,n);
x4_dot_data = zeros(m,n);
x5_dot_data = zeros(m,n);
for i = 1:m
    for j = 1:n
        x1 = x1_table_data(i,j);
        x2 = x2_table_data(i,j);
        x3 = x3_table_data(i,j);
        x4 = x4_table_data(i,j);
        x5 = x5_table_data(i,j);
        
        u1 = u1_table_data(i,j);
        u2 = u2_table_data(i,j);
        u3 = u3_table_data(i,j);
        
        x = [x1; x2; x3; x4; x5];
        u = [u1; u2; u3];

        [x_dot, y, ~] = diesel_engine(x, u, n_e_axis(j), model);
        M_e_data(i,j) = y.M_e;
        
        x1_dot_data(i,j) = x_dot(1);
        x2_dot_data(i,j) = x_dot(2);
        x3_dot_data(i,j) = x_dot(3);
        x4_dot_data(i,j) = x_dot(4);
        x5_dot_data(i,j) = x_dot(5);
    end
end

figure
surf(n_e_axis, M_e_axis, M_e_data)
title('M_e')
grid on

x_dot_titles = ["$\frac{d}{dt}p_{im}$", "$\frac{d}{dt}p_{em}$", ...
    "$\frac{d}{dt}X_{Oim}$", "$\frac{d}{dt}X_{Oem}$", "$\frac{d}{dt}\omega_{t}$"];
for i = 1:5
    h = figure;
    surf(n_e_axis, M_e_axis, x_dot_data(i:5:end, :));
    xlabel('Engine speed [rpm]')
    ylabel('Normalized torque [%]')
    zlabel(x_dot_titles(i), 'interpreter', 'latex');
    colorbar
    % set(h,'Units','Inches');
    % pos = get(h,'Position');
    % set(h,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
    % str = strcat('Figures/table_data_xdot', int2str(i));
    % print(h, str ,'-dpdf','-r0')
end

%% Plot engine map solutions

% max_torque_curve
% h = figure(1);
% hold on
% ylabel('Engine torque [Nm]')
% color = [0, 0.4470, 0.7410];
% p1 = plot(n_e_axis, M_e_data, '.', 'MarkerEdgeColor', color, 'MarkerFaceColor', color);
% % color = [0.8500, 0.3250, 0.0980];
% % p2 = plot(n_e_axis, M_e_data, '.', 'MarkerEdgeColor', color, 'MarkerFaceColor', color);
% axis([500 2000 0 2700])
% legend([p, p1(1), p2(1)], 'Maximum stationary torque curve', 'Stationary solutions', 'Relaxed solutions')

%% Export as PDF

% set(h,'Units','Inches');
% pos = get(h,'Position');
% set(h,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
% print(h,'Figures/engine_map_solutions','-dpdf','-r0')
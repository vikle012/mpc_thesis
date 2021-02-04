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

figure
grid on
surf(n_e_axis, M_e_axis, x1_table_data)
xlabel('Engine speed [rpm]')
ylabel('Normalized torque [%]')
zlabel('p_{im} [Pa]')
colorbar

figure
surf(n_e_axis, M_e_axis, x2_table_data)
xlabel('Engine speed [rpm]')
ylabel('Normalized torque [%]')
zlabel('p_{em} [Pa]')

figure
surf(n_e_axis, M_e_axis, x3_table_data)
xlabel('Engine speed [rpm]')
ylabel('Normalized torque [%]')
zlabel('X_{Oim} [-]')

figure
surf(n_e_axis, M_e_axis, x4_table_data)
xlabel('Engine speed [rpm]')
ylabel('Normalized torque [%]')
zlabel('X_{Oem} [-]')

figure
surf(n_e_axis, M_e_axis, x5_table_data)
xlabel('Engine speed [rpm]')
ylabel('Normalized torque [%]')
zlabel('\omega_{t} [rad/s]')

figure
surf(n_e_axis, M_e_axis, u1_table_data)
xlabel('Engine speed [rpm]')
ylabel('Normalized torque [%]')
zlabel('u_\delta [mg/cycle]')

figure
surf(n_e_axis, M_e_axis, u2_table_data)
xlabel('Engine speed [rpm]')
ylabel('Normalized torque [%]')
zlabel('u_{egr} [%]')

figure
surf(n_e_axis, M_e_axis, u3_table_data)
xlabel('Engine speed [rpm]')
ylabel('Normalized torque [%]')
zlabel('u_{vgt} [%]')

%% More plots

[m,n] = size(x1_table_data);
M_e_data = zeros(m,n);
x_dot_data = zeros(m*5,n);
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
        x_dot_data(i:i+4,j) = x_dot;
    end
end

figure
surf(n_e_axis, M_e_axis, M_e_data)
title('M_e')
grid on

x_dot_titles = ["x1dot", "x2dot", "x3dot", "x4dot", "x5dot"];
for i = 1:5
    figure
    surf(n_e_axis, M_e_axis, x_dot_data(i:5:end, :));
    title(x_dot_titles(i));
end
at_ne = 950;
index = 19;

[a, b, c] = find_reference_relax2(0, at_ne, model);

control.u_egract = 0; % without EGR-actuator dynamics
control.u_vgtact = 0; % without VGT-actuator dynamics

simU.n_e=[0 at_ne]; 
simU.u_delta=[0 u1_table_data(1,index)];
simU.u_egr=[0 u2_table_data(1,index)];        
simU.u_vgt=[0 u3_table_data(1,index)];

% --- Simulink initialization ---
model.p_im_Init          = x1_table_data(1,index);
model.p_em_Init          = x2_table_data(1,index);
model.X_Oim_Init         = x3_table_data(1,index);
model.X_Oem_Init         = x4_table_data(1,index);
model.omega_t_Init       = x5_table_data(1,index);
model.utilde_egr1_Init   = 0;
model.utilde_egr2_Init   = 0;
model.utilde_vgt_Init    = 0;

model.uInit_egr = u2_table_data(1,index);
model.uInit_vgt = u3_table_data(1,index);

X = [x1_table_data(1,index); x2_table_data(1,index); x3_table_data(1,index); x5_table_data(1,index); x5_table_data(1,index)];
U = [u1_table_data(1,index); u2_table_data(1,index); u3_table_data(1,index)];

[~,y,~] = diesel_engine(X, U, at_ne, model);
model.x_r_Init = y.x_r;
model.T_1_Init = y.T_1;
% -------------------------------

% Simulation
sim('TCDI_EGR_VGT', 6)

%% Plotting

x_data = [simp_im, simp_em, simX_Oim, simX_Oem, simn_t*pi/30];
x_titles = ["p_{im}", "p_{em}", "X_{Oim}", "X_{Oem}", "\omega_t"];

h = figure(1);
subplot(3,1,1)
yyaxis left
plot(tout, x_data(:,1))
ylabel('p_{im} [Pa]')
yyaxis right
plot(tout, x_data(:,2))
ylabel('p_{em} [Pa]')

subplot(3,1,2)
yyaxis left
plot(tout, x_data(:,3))
ylabel('X_{Oim} [-]')
yyaxis right
plot(tout, x_data(:,4))
ylabel('X_{Oem} [-]')

subplot(3,1,3)
plot(tout, x_data(:,5))
ylabel('\omega_t [rad/s]')
xlabel('Time [s]')

% set(h,'Units','Inches');
% pos = get(h,'Position');
% set(h,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
% print(h,'Figures/table_data_sim_x','-dpdf','-r0')

u_data = [simu_delta, simu_egr, simu_vgt];
u_titles = ["u_\delta", "u_{egr}", "u_{vgt}"];

h = figure(2);
subplot(2,1,1)
plot(tout, u_data(:,1))
ylabel('u_\delta [mg/cycle]')

subplot(2,1,2)
yyaxis left
plot(tout, u_data(:,2))
ylabel('u_{egr} [%]')
yyaxis right
plot(tout, u_data(:,3))
ylabel('u_{vgt} [%]')
xlabel('Time [s]')

% set(h,'Units','Inches');
% pos = get(h,'Position');
% set(h,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
% print(h,'Figures/table_data_sim_u','-dpdf','-r0')

% for i = 1:3
%     subplot(3,1,i)
%     hold on
%     plot(tout, u_data(:,i))
%     title(u_titles(i))
% end

h = figure;
hold on
plot(tout, simM_e)
ylabel('M_{e} [Nm]')
xlabel('Time [s]')

% set(h,'Units','Inches');
% pos = get(h,'Position');
% set(h,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
% print(h,'Figures/table_data_sim_Me','-dpdf','-r0')
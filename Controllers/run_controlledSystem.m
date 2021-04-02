clear
clc

% Change to correct path if needed
addpath('C:\Users\Jonte\Documents\GitHub\mpc_thesis\MATLAB_model')
addpath('C:\Users\Jonte\Documents\GitHub\mpc_thesis\Controllers')
addpath('C:\Users\Jonte\Documents\GitHub\mpc_thesis\Lookup_tables')
addpath('C:\Users\Jonte\Documents\GitHub\mpc_thesis\Lookup_tables\data')
addpath('C:\Users\Jonte\Documents\GitHub\mpc_thesis\WahlstromErikssonTCDI_EGR_VGT')
addpath('C:\Users\Jonte\Documents\GitHub\mpc_thesis\CasADi\casadi-windows-matlabR2016a-v3.5.5')
load parameterData

%% System Inputs (dimensions of vectors must match)

% WHTC
[torque, speed, time] = interpolate_WHTC;
M_e_input = torque;
n_e_input = speed;
time_vector = time;

% Transient response: needs different reference plotting
% M_e_input = [650; 2000];
% n_e_input = [1200; 1200];
% time_vector = [0 1]';
% stop_at = 10; % Only for transient

% Sinusoid response
% M_e_input = 1000 + 500*sin(0:pi/200:5*pi)';
% n_e_input = repmat(1200,1,length(M_e_input));
% t_end = (length(M_e_input)-1)*0.01;
% time_vector = [0:0.01:t_end]';

% Ramp response
% M_e_input = [650 650:2:2000]';
% n_e_input = repmat(1200,1,length(M_e_input));
% t_end = (length(M_e_input)-2)*0.01+0.5;
% time_vector = [0 0.5:0.01:t_end]';

%% Reference Generating

[x_ref, u_ref] = reference_generator(M_e_input, n_e_input);
% load('WHTC_ref') pre-calculated WHTC references to save time

%% Select Controller

% Simulation times
start_time = time_vector(1);
stop_time = time_vector(end);

% For transient
if exist('stop_at')
   stop_time = stop_at; 
end

controller_object = 'SLpreview_current';
% save_name = 'longPreview_ref';
plot_ref = 1;

integral_action = 0;
% 0: without integral action
% 1: with integral action

control.u_egract = 0; % without EGR-actuator dynamics
control.u_vgtact = 0; % without VGT-actuator dynamics

%% Tuning Parameters

% Parameters required in set-up
global init_param;
init_param.model = model;
init_param.integral_action = integral_action;
init_param.T_s = 0.01;
init_param.N = 40;

% State weights
q1 = 10/((0.5*model.p_amb + 10*model.p_amb)/2)^2;
q2 = 10/((0.5*model.p_amb + 20*model.p_amb)/2)^2;
q3 = 1/((0 + 1)/2)^2;
q4 = 1/((0 + 1)/2)^2;
q5 = 10/((100*pi/30 + 200000*pi/30)/2)^2;

% Control signal weights
r1 = 5/((1 + 250)/2)^2;
r2 = 0.05/((0 + 100)/2)^2;
r3 = 0.05/((20 + 100)/2)^2; 

% Integral action weights
s1 = 0.05/((1 + 250)/2)^2;
s2 = 0.05/((0 + 100)/2)^2;
s3 = 0.05/((20 + 100)/2)^2; 

% No need to modify these
init_param.Q = diag([q1 q2 q3 q4 q5]);
init_param.R = diag([r1 r2 r3]);
init_param.S = diag([s1 s2 s3]);

init_param.u_old = u_ref(:,1);

%% Simulation Set-up

simulate.T_s = init_param.T_s;

if or(strcmp(controller_object, "SLpreview_ref"), ...
        or(strcmp(controller_object, "SLpreview_current"), ...
        strcmp(controller_object, "Lpreview")))
    % Not proud of this solution
    x_ref_all = [];
    u_ref_all = [];
    for i = 1:length(time_vector)
        t1 = time_vector(i);
        if i == length(time_vector)
            t2 = stop_time;
            repeat = uint64((t2 - t1)/simulate.T_s + 1);
        else
            t2 = time_vector(i+1);
            repeat = uint64((t2 - t1)/simulate.T_s);
        end
        x_ref_all = [x_ref_all repmat(x_ref(:,i), 1, repeat)];
        u_ref_all = [u_ref_all repmat(u_ref(:,i), 1, repeat)];
    end
    % Extend further
    x_ref_all = [x_ref_all repmat(x_ref(:,end), 1, init_param.N)];
    u_ref_all = [u_ref_all repmat(u_ref(:,end), 1, init_param.N)];
    simulate.n_e = timeseries(n_e_input, time_vector); % Row vector since "From Workspace"
    simulate.x_ref = timeseries(x_ref_all, 0);
    simulate.u_ref = timeseries(u_ref_all, 0); 
else
    simulate.x_ref = timeseries(x_ref, time_vector); % Row vector since "From Workspace"
    simulate.u_ref = timeseries(u_ref, time_vector);
    simulate.n_e = timeseries(n_e_input, time_vector);
end

% --- Simulink initialization ---
simulate.p_im_Init          = x_ref(1,1);
simulate.p_em_Init          = x_ref(2,1);
simulate.X_Oim_Init         = x_ref(3,1);
simulate.X_Oem_Init         = x_ref(4,1);
simulate.omega_t_Init       = x_ref(5,1);
simulate.utilde_egr1_Init   = 0;
simulate.utilde_egr2_Init   = 0;
simulate.utilde_vgt_Init    = 0;

% Not used
simulate.u_egr_Init = u_ref(2,1);
simulate.u_vgt_Init = u_ref(3,1);

[~,y,~] = diesel_engine(x_ref(:,1), u_ref(:,1), n_e_input(1), model);
simulate.x_r_Init = y.x_r;
simulate.T_1_Init = y.T_1;
% -------------------------------

% Open system
open_system('controlledSystem')

% Simulink controller set-up
set_param('controlledSystem/MPC/MATLAB System', 'System', ...
    controller_object, 'SimulateUsing', 'Interpreted execution');
set_param('controlledSystem', 'SimulationCommand', 'Update');

% Simulation
simOut = sim('controlledSystem', 'StartTime', num2str(start_time),...
                                 'StopTime', num2str(stop_time));

%% Plotting

x_data = [simOut.simp_im, simOut.simp_em, simOut.simX_Oim,...
          simOut.simX_Oem, simOut.simn_t*pi/30];
x_titles = ["p_{im} [Pa]", "p_{em} [Pa]", "X_{Oim} [-]",...
            "X_{Oem} [-]", "\omega_t [rad/s]"];

h1 = figure(1);
for i = 1:5
    subplot(5,1,i)
    hold on
    if plot_ref == 1
        stairs(time_vector, x_ref(i,:), 'k:');      
        plot(simOut.simTime, x_data(:,i))
    else
        plot(simOut.simTime, x_data(:,i), '--')
    end
    
    ylabel(x_titles(i))
end
xlabel('Time [s]')

u_delta = squeeze(simOut.simu_delta);
u_egr = squeeze(simOut.simu_egr);
u_vgt = squeeze(simOut.simu_vgt);

u_data = [u_delta, u_egr, u_vgt];
u_titles = ["u_\delta [mg/cycle]", "u_{egr} [%]", "u_{vgt} [%]"];

h2 = figure(2);
for i = 1:3
    subplot(3,1,i)
    hold on
    if plot_ref == 1
        stairs(time_vector, u_ref(i,:), 'k:');
        stairs(simOut.simTime, u_data(:,i))
    else
        stairs(simOut.simTime, u_data(:,i), '--')
    end
    ylabel(u_titles(i))
end
xlabel('Time [s]')

M_e = squeeze(simOut.simM_e);
lambda_O = squeeze(simOut.simlambda);

h4 = figure(4);
subplot(2,1,1)
hold on
if plot_ref == 1
    plot(simOut.simTime, lambda_O)
else
    plot(simOut.simTime, lambda_O, '--')
end
ylabel('\lambda_O [-]')
subplot(2,1,2)
hold on
if plot_ref == 1
    plot(simOut.simTime, simOut.simx_egr)
else
    plot(simOut.simTime, simOut.simx_egr, '--')
end
ylabel('x_{egr} [-]')
xlabel('Time [s]')

h3 = figure(3);
hold on
if plot_ref == 1
    stairs(time_vector, M_e_input, 'k:')
    plot(simOut.simTime, M_e)
else
    plot(simOut.simTime, M_e, '--')
    
    legend(["Reference", "Without preview", "With preview"], 'Location', 'southeast')
end
ylabel('M_e [Nm]')
xlabel('Time [s]')

%% Save workspace

% save(save_name)

%% Save figures

% set(h1,'Units','Inches');
% pos = get(h1,'Position');
% set(h1,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
% str = strcat(save_name, '_x');
% print(h1, strcat('Figures/Results/', str),'-dpdf','-r0')
% 
% set(h2,'Units','Inches');
% pos = get(h2,'Position');
% set(h2,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
% str = strcat(save_name, '_u');
% print(h2, strcat('Figures/Results/', str),'-dpdf','-r0')
% 
% set(h3,'Units','Inches');
% pos = get(h3,'Position');
% set(h3,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
% str = strcat(save_name, '_Me');
% print(h3, strcat('Figures/Results/', str),'-dpdf','-r0')
% 
% set(h4,'Units','Inches');
% pos = get(h4,'Position');
% set(h4,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
% str = strcat(save_name, '_lambda_xegr');
% print(h4, strcat('Figures/Results/', str),'-dpdf','-r0')

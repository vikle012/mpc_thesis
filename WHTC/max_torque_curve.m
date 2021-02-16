%% Generates a maximum torque curve for stationary points

load('parameterData')

M_e = [];
N_e = [];
W_opt = [];

for n_e = 500:25:2000

    X = casadi.SX.sym('X',5); % 5x1 matrix
    U = casadi.SX.sym('U',3); % 3x1 matrix

    [x_dot, y, constr] = diesel_engine(X, U, n_e, model);

    f = -y.M_e;                        % Objective/cost function
    w = [X; U];                        % Optimization variables

    % Initial values/guess
    if n_e == 800 || n_e == 825
        w0 = [150437.593727065;160060.761239722;0.221915115974491;0.0956202415346445;5642.68013104888; 131.017038028082;14.0825363615519;24.0798004681596];
    else
        w0 = [1.0e+05*1.9307; 1.0e+05*2.1069; 0.2120; 0.1109; 7.5804e+03; 127.4860; 63.2446; 40.5904];
    end
    
    % Constraints on opt. variables
    lbw = [0.5*model.p_amb; 0.5*model.p_amb; 0; 0; 100*pi/30; 1; 0; 20]; 
    ubw = [10*model.p_amb; 20*model.p_amb; 1; 1; 200000*pi/30; 250; 100; 100];

    % Constraint
    g = [x_dot; constr.constraints];        
    lbg = [zeros(5,1); constr.lbg];            
    ubg = [zeros(5,1); constr.ubg];

    % Create solver
    nlp = struct('x', w, 'f', f, 'g', g);
    solver = casadi.nlpsol('solver', 'ipopt', nlp);

    solution = solver('x0',  w0, ...      % Initial guess
              'lbx', lbw, ...     % Lower variable bound
              'ubx', ubw, ...     % Upper variable bound
              'lbg', lbg, ...     % Lower constraint bound
              'ubg', ubg);        % Upper constraint bound

    w_opt = full(solution.x);
    W_opt = [W_opt w_opt];
    M_e = [M_e; -full(solution.f)];
    N_e = [N_e; n_e];
end

%% Plot

h = figure(1);
hold on
p = plot(N_e, M_e, 'k');
% plot(N_e, M_e, 'x') 
xlabel('Engine speed [rpm]')
ylabel('Maximum engine torque [Nm]')


%% Export as PDF

% set(h,'Units','Inches');
% pos = get(h,'Position');
% set(h,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
% print(h,'Figures/M_max_n','-dpdf','-r0')
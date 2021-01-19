
M_e = [];
N_e = [];

for n_e = 500:25:2000

    X = casadi.SX.sym('X',5); % 5x1 matrix
    U = casadi.SX.sym('U',3); % 3x1 matrix

    [x_dot, y, constr] = diesel_engine(X, U, n_e, model);

    f = -y.M_e;                        % Objective/cost function
    w = [X; U];                        % Optimization variables

    % Initial values/guess
    w0 = [1.0e+05*1.9307; 1.0e+05*2.1069; 0.2120; 0.1109; 7.5804e+03; 127.4860; 63.2446; 40.5904];
    
    if n_e == 800 || n_e == 825
        w0 = [150437.593727065;160060.761239722;0.221915115974491;0.0956202415346445;5642.68013104888; 131.017038028082;14.0825363615519;24.0798004681596];
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
    x_opt = w_opt(1:5);
    u_opt = w_opt(6:8);
    M_e = [M_e; -full(solution.f)];
    N_e = [N_e; n_e];
end

figure(1)
plot(N_e, M_e)
% axis([0 2500 0 2500])
xlabel('Engine speed [rpm]')
ylabel('Engine torque [Nm]')
title('Torque speed map')

figure(2)
plot(N_e, M_e.*N_e*pi/30)
% axis([0 2500 0 2500])
xlabel('Engine speed [rpm]')
ylabel('Maximum power [W]')
title('Power speed map')
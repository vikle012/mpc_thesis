function [x_opt, u_opt, dx, y] = find_reference_relax2(M_e, n_e, model)

% Succesively relaxes the optimization problem by lowering the weight on
% the derivative in the cost function until a solution is found

% Declare states
x = casadi.SX.sym('X',5); % 5x1 matrix
% p_im    = x(1);
% p_em    = x(2);
% X_Oim   = x(3);
% X_Oem   = x(4);
% w_t     = x(5);

% Declare control signals
u = casadi.SX.sym('U',3); % 3x1 matrix
% u_delta = u(1);
% u_egr   = u(2);
% u_vgt   = u(3);

% Weight matrices
q1 = 1/((0.5*model.p_amb + 10*model.p_amb)/2)^2;
q2 = 1/((0.5*model.p_amb + 20*model.p_amb)/2)^2;
q3 = 0.001/((0 + 1)/2)^2;
q4 = 0.001/((0 + 1)/2)^2;
q5 = 1/((100*pi/30 + 200000*pi/30)/2)^2;

r1 = 100000/((1 + 250)/2)^2;
r2 = 1/((0 + 100)/2)^2;
r3 = 1/((20 + 100)/2)^2;

s1 = 100000000/((0.5*model.p_amb + 10*model.p_amb)/2)^2;
s2 = 100000000/((0.5*model.p_amb + 20*model.p_amb)/2)^2;
s3 = 1000000/((0 + 1)/2)^2;
s4 = 1000000/((0 + 1)/2)^2;
s5 = 100000000/((100*pi/30 + 200000*pi/30)/2)^2;

Q = diag([q1 q2 q3 q4 q5]);
R = diag([r1 r2 r3]);
S = diag([s1 s2 s3 s4 s5]);

% Optimization variables
w = [x; u];

% Initial values/guess
w0  = [1.0e+05*1.9307; 1.0e+05*2.1069; 0.2120; 0.1109; 7.5804e+03; ...
       127.4860; 63.2446; 40.5904]; 

% Constraints on opt. variables
lbw = [0.5*model.p_amb; 0.5*model.p_amb; 0; 0; 100*pi/30; 1; 0; 20]; 
ubw = [10*model.p_amb; 20*model.p_amb; 1; 1; 200000*pi/30; 250; 100; 100];

flag = 0;
counter = 1;
while flag == 0
    [x_dot, y, constr] = diesel_engine(x, u, n_e, model);

    S = S/10;
    f = x'*Q*x + u'*R*u + x_dot'*S*x_dot;

    % Constraint
    g = [y.M_e; constr.constraints];       
    lbg = [M_e; constr.lbg];            
    ubg = [inf; constr.ubg];

    % Create solver
    nlp = struct('x', w, 'f', f, 'g', g);
    solver = casadi.nlpsol('solver', 'ipopt', nlp);

    solution = solver('x0',  w0, ...      % Initial guess
              'lbx', lbw, ...     % Lower variable bound
              'ubx', ubw, ...     % Upper variable bound
              'lbg', lbg, ...     % Lower constraint bound
              'ubg', ubg);        % Upper constraint bound
    if solver.stats.success
        w_opt = full(solution.x);
        x_opt = w_opt(1:5);
        u_opt = w_opt(6:8);
        [dx, y, ~] = diesel_engine(w_opt(1:5), w_opt(6:end), 500, model);
        flag = 1;
    else
        flag = 0;
        counter = counter + 1;
    end
    
    if counter > 7
        x_opt = ones(5,1)*nan;
        u_opt = ones(3,1)*nan;
        dx = ones(5,1)*nan;
        y = nan;
        break
    end
end 





end
function [x_opt, u_opt, success] = find_reference(M_e, n_e, param)

% TEST: [x, u] = find_trajectory([800; 900], [1500; 1500], model)

if length(M_e) ~= length(n_e)
    error('Unmatching dimensions')
else
    x_opt = [];
    u_opt = [];
    success = [];
    
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
    q1 = 1/((0.5*param.p_amb + 10*param.p_amb)/2)^2;
    q2 = 1/((0.5*param.p_amb + 20*param.p_amb)/2)^2;
    q3 = 0.001/((0 + 1)/2)^2;
    q4 = 0.001/((0 + 1)/2)^2;
    q5 = 1/((100*pi/30 + 200000*pi/30)/2)^2;

    r1 = 100000/((1 + 250)/2)^2;
    r2 = 1/((0 + 100)/2)^2;
    r3 = 1/((20 + 100)/2)^2;

    Q = diag([q1 q2 q3 q4 q5]);
    R = diag([r1 r2 r3]);
    
    % Optimization variables
    w = [x; u];                                
    
    % Initial values/guess
    w0  = [1.0e+05*1.9307; 1.0e+05*2.1069; 0.2120; 0.1109; ...
           7.5804e+03; 127.4860; 63.2446; 40.5904]; 
    
    % Constraints on opt. variables
    lbw = [0.5*param.p_amb; 0.5*param.p_amb; 0; 0; 100*pi/30; 1; 0; 20]; 
    ubw = [10*param.p_amb; 20*param.p_amb; 1; 1; 200000*pi/30; 250; 100; 100];
    
    for i = 1:length(M_e)
        [x_dot, y, constr] = diesel_engine(x, u, n_e(i), param);
        
        % Objective/cost function
        f = x'*Q*x + u'*R*u;                        
        
        % Constraint
        g = [y.M_e; x_dot; constr.constraints];       
        lbg = [M_e(i); zeros(5,1); constr.lbg];            
        ubg = [inf; zeros(5,1); constr.ubg];

        % Create solver
        nlp = struct('x', w, 'f', f, 'g', g);
        solver = casadi.nlpsol('solver', 'ipopt', nlp);
        
        solution = solver('x0',  w0, ...      % Initial guess
                  'lbx', lbw, ...     % Lower variable bound
                  'ubx', ubw, ...     % Upper variable bound
                  'lbg', lbg, ...     % Lower constraint bound
                  'ubg', ubg);        % Upper constraint bound

        w_opt = full(solution.x);
        x_opt = [x_opt w_opt(1:5)];
        u_opt = [u_opt w_opt(6:8)];
        success = [success; solver.stats.success];
    end 
end

end


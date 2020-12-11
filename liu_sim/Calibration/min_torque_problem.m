function calProb = min_torque_problem(N_ice, param)

% Variables
nx = 4; % No. of states
nu = 3; % No. of controls
np = 0; % No. of parameters

% Symbolic variables
[~, X, U, ~] = yopVar('t', nx, nu, np);

% Model calculations
[xdot, h, signals] = liu_diesel_2(X, U, N_ice, param);

% Positive torque required: M_ice > 0.0001
h = [h; 0.1-signals.M_ice];

% Steady-state conditions
g = xdot;  % Steady-state

calProbCell={...
    'J', signals.M_ice ...       Objective fn (min torque)
    'f', xdot ...            Dynamics
    'x_min', param.constr.x_min ... State lower bound
    'x_max', param.constr.x_max ... State upper bound
    'u_min', param.constr.u_min ... Control lower bound
    'u_max', param.constr.u_max ... Control upper bound
    'h', h, ...              Inequality constraints
    'g', g, ...              Equality constraints
    };
        
calProb = yopSPP(calProbCell);

end
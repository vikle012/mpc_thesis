function calProb = engine_min_fuel_calibration_NOx(N_ice, M_ice, param)

% Variables
nx = 7; % No. of states
nu = 3; % No. of controls
np = 0; % No. of parameters

% Symbolic variables
[~, x, u, ~] = yopVar('t', nx, nu, np);

% Model calculations
[xdot, h, c] = LD2_pwl(x, u, N_ice, param);

% Steady-state conditions
g = [xdot; c.M_ice-M_ice];

% Control derivatives zero
param.constr.u_min = zeros(3,1);
param.constr.u_max = zeros(3,1);

calProbCell={...
    'J', c.W_f ...       Objective fn (min fuel)
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
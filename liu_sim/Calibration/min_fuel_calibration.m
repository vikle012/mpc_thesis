function calProb = min_fuel_calibration(N_ice, M_ice, param)

% Variables
nx = 4; % No. of states
nu = 3; % No. of controls
np = 0; % No. of parameters

% Symbolic variables
[~, X, U, ~] = yopVar('t', nx, nu, np);

% Model calculations
[xdot, h, signals] = liu_diesel_2(X, U, N_ice, param);

% Steady-state conditions
g = xdot; %[xdot; signals.M_ice-M_ice];
h = [h; signals.M_ice-(M_ice+4); (M_ice-4)-signals.M_ice];

calProbCell={...
    'J', signals.W_f ...        Objective fn (min fuel)
    'f', xdot ...                   Dynamics
    'x_min', param.constr.x_min ... State lower bound
    'x_max', param.constr.x_max ... State upper bound
    'u_min', param.constr.u_min ... Control lower bound
    'u_max', param.constr.u_max ... Control upper bound
    'h', h, ...                     Inequality constraints
    'g', g, ...                     Equality constraints
    };
        
calProb = yopSPP(calProbCell);

end
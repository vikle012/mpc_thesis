function ocp = test_tipin_ocp(x0, u0, xT, uT, N_ice, param)

% Since the optimization is piecewise linear (improves optimization
% performance) the original state and control signals need to be converted
% to the pwl set of states and controls.
x0_pwl = [x0; u0]; % Initial state
xT_pwl = [xT; uT]; % Terminal state

% Variables
nx = 8; % No. of states
nu = 3; % No. of controls
np = 0; % No. of parameters

% Symbolic variables
[~, x, u, ~] = yopVar('t', nx, nu, np);

% Model calculations
[xdot, h, signals] = test_LD2_pwl(x, u, N_ice, param);

% Control derivative zero at boundary points
gi = u; 
gf = u; 

param.constr.x_min = [param.constr.x_min(1:4); 473; param.constr.x_min(5:end)];
param.constr.x_max = [param.constr.x_max(1:4); 5000; param.constr.x_max(5:end)];

phaseData={'L', 1         ... Integral cost
            'tf_min', 0.1 ... End time lower bound
            'tf_max', 5   ... End time upper bound
            'f', xdot     ... Dynamics
            'xi_min', x0_pwl ...  Initial state lower bound
            'xi_max', x0_pwl ...  Initial state upper bound
            'xf_min', xT_pwl ...  Final state lbd
            'xf_max', xT_pwl ...  Final state ubd
            'x_min', param.constr.x_min ...  State lower bound
            'x_max', param.constr.x_max ...  State upper bound
            'u_min', param.constr.u_min ...  Control lower bound
            'u_max', param.constr.u_max ...  Control upper bound
            'h', h   ... Inequality constraints
            'hi', h  ... Initial inequality constraints
            'gi', gi ... Initial equality constraint
            'gf', gf ... Terminal equality constraint
            };
        
p1 = yopPhase(phaseData);

% Create complete OCP
ocpDesc={'state_norm', [param.ocp.state_norm(1:4); 2000; param.ocp.state_norm(5:end)], ...      % State normlization
         'control_norm', param.ocp.control_norm, ...  % Control normalization
         'obj_norm', 1};              % Obj. fn normalization

ocp = yopOcp(ocpDesc, p1);

end
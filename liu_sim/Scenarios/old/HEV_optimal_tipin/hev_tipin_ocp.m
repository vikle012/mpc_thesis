function ocp = hev_tipin_ocp(x0, u0, xT, uT, tf, param)

% Since the optimization is piecewise linear (improves optimization
% performance) the original state and control signals need to be converted
% to the pwl set of states and controls.

x0 = [x0; u0]; % Initial state
xT = [xT; uT]; % Terminal state

% Variables
nx = 11; % No. of states
nu = 4; % No. of controls
np = 0; % No. of parameters

[t, x, u, ~] = yopVar('t', nx, nu, np);

[xdot, h, signals] = hybridPowertrain_pwl(x, u, t, param);


gi = u; 
gf = u; 


phaseData={'L', signals.W_f         ... Integral cost (1 = time)
            'tf_min', tf ... End time lower bound
            'tf_max', tf ... End time upper bound
            'f', xdot     ... Dynamics
            'xi_min', x0(:,1) ...  Initial state lower bound
            'xi_max', x0(:,2) ...  Initial state upper bound
            'xf_min', xT(:,1) ...  Final state lbd
            'xf_max', xT(:,2) ...  Final state ubd
            'x_min', param.ld2.constr.x_min ...  State lower bound
            'x_max', param.ld2.constr.x_max ...  State upper bound
            'u_min', param.ld2.constr.u_min ...  Control lower bound
            'u_max', param.ld2.constr.u_max ...  Control upper bound
            'h', h   ... Inequality constraints
            'hi', h  ... Initial inequality constraints
            'gi', gi ... Initial equality constraint
            'gf', gf ... Terminal equality constraint
            };

p1 = yopPhase(phaseData);

ocpDesc={'state_norm', param.ld2.ocp.state_norm, ...      % State normlization
         'control_norm', param.ld2.ocp.control_norm, ...  % Control normalization
         'obj_norm', 1};              % Obj. fn normalization

ocp = yopOcp(ocpDesc, p1);






end


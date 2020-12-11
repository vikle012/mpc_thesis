function [ocp, L, E] = ocp__hev_eats_whtc(x0, u0, xT, uT, t0, tf, param)
% FUNCTION: Creates an optimal control problem 'ocp' to be used in the
%           optimization.
%
% SYNTAX: [output1] = ocp__hev_whtc(input1, input2, input3, input4, ...
%                                   input5, input6, input7);
%
% INPUTS:
%   input1 - Initial state vector
%   input2 - Initial control vector
%   input3 - Terminal state vector
%   input4 - Terminal control vector
%   input5 - Start time
%   input6 - Terminal time
%   input7 - Struct with all parameters
%
% OUTPUTS:
%   output1 - Optimal control proplem struct
%
% EXAMPLE:
%   ocp = ocp__hev_whtc(x0, u0, xT, uT, 0, 1800, param)
%
% OTHER FILES REQUIRED:
%   .m files:
%       PT_PWL__hev_whtc
%       yopPhase
%       yopOcp
%
%   .mat files:
%       none
%
% SUBFUNCTIONS:
%   none
%
% Author: Hampus Andersson and Fredrik Andersson
% April 2018; Last revision: 13-April-2018

%------------- BEGIN CODE --------------

% Since the optimization is piecewise linear (improves optimization
% performance) the original state and control signals need to be converted
% to the pwl set of states and controls.

x0 = [x0; u0]; % Initial state
xT = [xT; uT]; % Terminal state

% Variables
nx = 23; % No. of states
nu = 4; % No. of controls
np = 0; % No. of parameters

[t, x, u, ~] = yopVar('t', nx, nu, np);

[xdot, h, signals] = PT_PWL__hev_eats_whtc(x, u, t, param, t0);

gi = u;
gf = u;

% 'L' is the objective function that is minimized for each step in the
% optimizer. 'E' is the objective function for the terminal values.

L = 'NOx_Cu';
E = 'null';
%lambda = 1.84;
phaseData={'L', signals.NOx_TP_Cu ...signals.W_f ...signals.W_f.*param.ld2.q_HV + lambda.*signals.P_em_elec ... abs(250 - signals.T_aSCR)...signals.NOx_TP ...signals.W_f ...+ signals.W_f  ... Integral cost (1 = time)
    ...'E', signals.NOx_tot ...
    'tf_min', tf-t0 ... End time lower bound
    'tf_max', tf-t0 ... End time upper bound
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

ocpDesc={'state_norm', param.ld2.ocp.state_norm, ... % State normlization
    'control_norm', param.ld2.ocp.control_norm, ... % Control normalization
    'obj_norm', 100}; % Obj. fn normalization

ocp = yopOcp(ocpDesc, p1);

%------------- END OF CODE --------------
end


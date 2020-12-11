function [u_f, M_em] = ECMS_minH__hev_whtc(lambda, X, t, param)
% FUNCTION: Optimal selection of m_f and M_em through minimization of
%           hamiltonian according to ECMS methodology.
%
%           Minimize hamiltonian with respect to WHTC following
%           H  = Pf + lambda*Pech
%           u* = arg min H
%
% OTHER FILES REQUIRED:
%   .m files:
%       SOCtoUoc
%       engine_torque
%
%   .mat files:
%       none
%
% SUBFUNCTIONS:
%   WHTC_M_nonlcon
%
% Author: Hampus Andersson and Fredrik Andersson
% April 2018; Last revision: 04-April-2018
%
%
% EQUATIONS (( only for show, used in 'hamiltonian = @(x) ...' ))
%
% % ELECTRIC
% P_loss_lut = param.em.ploss_lut;
% P_em_elec = w .* M_em + full(param.em.ploss_lut([w M_em]));
% U_oc    = SOCtoUoc(SOC, param.batpack);
% U_batt  = U_oc./2 + sqrt((U_oc.^2)./4 - P_em_elec.*param.batpack.R);
% I_batt = P_em_elec ./ U_batt;
% P_ech = I_batt .* U_oc;
%
% % ICE
% W_f     = (1e-6/120) .* u_f .* N .* param.ld2.n_cyl; % Fuel massflow [kg/s]
% Pf = W_f .* param.ld2.q_HV;

%------------- BEGIN CODE --------------

% For easy indexing
p_im = X(2);
p_em = X(3);
N    = X(6);
SOC  = X(7);

% Hamiltonian to be minimized through selection of m_f and M_em. All
% equations previously stated are combined into one
[u_f, M_em] = enum(2);
hamiltonian = @(x,lambda,N,SOC,param) (1e-6/120) .* x(u_f) .* N .* ...
    param.ld2.n_cyl .* param.ld2.q_HV + lambda .* (N*pi/30 .* x(M_em) + ...
    full(param.em.ploss_lut([N*pi/30 x(M_em)]))) ./ ...
    (SOCtoUoc(SOC, param.batpack)./2 + ...
    sqrt((SOCtoUoc(SOC, param.batpack).^2)./4 - ...
    (N*pi/30 .* x(M_em) + full(param.em.ploss_lut([N*pi/30 x(M_em)]))).* ...
    param.batpack.R)) .* SOCtoUoc(SOC, param.batpack);

% Initial guess (( Most stable it seems at [0, 0] ))
x0 = [0, 0];

% No linear constraints
A   = [];
b   = [];
Aeq = [];
beq = [];

% Lower and upper bounds (( u_f: [0, 280], M_em: [-1500, 1500] ))
lb  = [param.ld2.constr.x_min(8), param.ld2.constr.x_min(11)];
ub  = [param.ld2.constr.x_max(8), param.ld2.constr.x_max(11)];

% Optimization options passed to 'fmincon'
options = optimoptions('fmincon','Display','off');%,'Algorithm','sqp');

% Solve for optimal controls u* that minimizing the hamiltonian H (fun).
% Non-linear constraint on following torque profile in 'WHTC_M'.
U = fmincon(@(x)hamiltonian(x,lambda,N,SOC,param), ...
    x0, A, b, Aeq, beq, lb, ub, @(x)WHTC_M_nonlcon(x,t,N,p_im,p_em,param), options);

u_f = U(1);
M_em = U(2);

    function [c,ceq] = WHTC_M_nonlcon(U, t, N, p_im, p_em, param)
        % Shall be less than zero, give at least required torque
        lut_M = param.WHTC.lut_M;
        M_ice = engine_torque(N, p_im, p_em, U(1), param.ld2); % [Nm]
        c = full(lut_M(t)) - (U(2) + M_ice);
        % None 'equal non-linear' constraint
        ceq = [];
    end

%------------- END OF CODE --------------
end
function [M_em_maxdiff, M_em_mindiff] = M_em_max_fn(M_em, N_em)
% Calculates differences M-Mmax and Mmin-M.
%   Maximum torques approximated by polynomial fitted to data from electric
%   motor map.

% Line + 3rd order polynomials for max and min torques.
M_em_maxdiff = M_em.*[1;1] - [-0.0161250784334023.*N_em + 1491.55024275362 ; ... % Line
                           -1.23101738450936e-07 .*N_em.^3 + ... % 3rd order poly
                            0.00103974588036777  .*N_em.^2 + ...
                           -3.12619953102183     .*N_em + ...
                            3701.60096038325];
end


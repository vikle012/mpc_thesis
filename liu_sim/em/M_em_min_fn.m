function [M_em_mindiff] = M_em_min_fn(M_em, N_em)
% Calculates differences M-Mmax and Mmin-M.
%   Maximum torques approximated by polynomial fitted to data from electric
%   motor map.

% Line + 3rd order polynomials for max and min torques.                     
M_em_mindiff = [-0.0152828102766799 .*N_em - 1499.31228926219 ; ... % Line
                6.03630766905148e-08    .*N_em.^3 - ... % 3rd order poly
                 0.000641064825511844   .*N_em.^2 + ...
                 2.33358948727600       .*N_em - ... 
                 3336.73041264945] - M_em.*[1;1];
end


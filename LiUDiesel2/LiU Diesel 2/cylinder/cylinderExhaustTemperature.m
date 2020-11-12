function [ T_cyl_out ] = cylinderExhaustTemperature(p_im, p_em, T_im, W_f, W_a, param)
% -------------------------------------------------------------------------
% Copyright 2018, Kristoffer Ekberg, Viktor Leek, Lars Eriksson
%
% This file is part of LiU Diesel 2.
%
% LiU Diesel 2 is free software: you can redistribute it and/or modify it
% under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or any
% later version.
% LiU Diesel 2 is distributed in the hope that it will be useful, but
% WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
% Public License for more details.
%
% You should have received a copy of the GNU General Public License
% along with LiU Diesel 2.  If not, see <https://www.gnu.org/licenses/>.
% -------------------------------------------------------------------------
q_in = W_f .* param.q_HV ./ (W_f + W_a);
T_cyl_out = param.eta_sc .* (p_em./p_im).^(1 - 1/param.gamma_air) .* param.r_c^(1 - param.gamma_air) .*...
    (q_in/param.c_pa + T_im .* param.r_c^(param.gamma_air - 1));

end
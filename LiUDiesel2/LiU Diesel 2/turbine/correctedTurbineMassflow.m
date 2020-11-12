function [W_t_corr, k0, k1, k2] = correctedTurbineMassflow(PiT, N_t_corr, param)
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
N_t_norm = N_t_corr./1000;

k00 = param.turb.k0(1);
k02 = param.turb.k0(2);

k10 = param.turb.k1(1);
k11 = param.turb.k1(2);

k20 = param.turb.k2(1);
k21 = param.turb.k2(2);
k22 = param.turb.k2(3);

k0 = k02*N_t_norm.^2                + k00;
k1 =                   k11*N_t_norm + k10;
k2 = k22*N_t_norm.^2 + k21*N_t_norm + k20;

W_t_corr = k0.*(1 - PiT.^k1).^k2;

end
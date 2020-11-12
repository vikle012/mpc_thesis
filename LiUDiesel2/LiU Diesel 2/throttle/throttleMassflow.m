function [W_thr, Pi] = throttleMassflow(pus, pds, T_c, u_thr, param)
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
Pi = pds./pus;
Psi = PsiOhata(Pi, param.gamma_air);
W_thr = pus./sqrt(param.R_a.*T_c) .* param.throttle.Cd .* param.throttle.A .* u_thr .* Psi;
end
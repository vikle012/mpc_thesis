function [W_wg, Psi, Pi]= wastegateMassflow( pus, pds, T_em, u_wg, ice_param )
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
Psi = PsiOhata(Pi, ice_param.gamma_e);
W_wg = pus./sqrt(ice_param.R_e.*T_em) * ice_param.wastegate.A .* ice_param.wastegate.Cd .* u_wg .* Psi;
end
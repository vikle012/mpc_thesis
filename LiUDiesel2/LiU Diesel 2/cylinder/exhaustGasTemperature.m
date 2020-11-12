function [T_em, T_e] = exhaustGasTemperature(p_im, p_em, T_im, W_f, W_cyl, param)
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
% Cylinder out temperature
T_e = cylinderExhaustTemperature(p_im, p_em, T_im, W_f, W_cyl, param);

% Exhaust manifold temperature
T_em = exhaustManifoldTemperature(T_e, W_f, W_cyl, param); %[K]

end
function [eta_ig, eta_otto, gamma_cyl, eta_cal, c_cal_1, c_cal_2] = indicatedEfficiency(phi, N_ice, u_f, param)
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
%% Calc gamma
gamma_cyl = param.c_gamma(1) + param.c_gamma(2)*phi + param.c_gamma(3)*phi.^2;

% Calc operating point dependent engine calibration factor
c_cal_1 = param.c_inj(2) + param.c_inj(3)*(N_ice/1000);
c_cal_2 = param.c_inj(4) + param.c_inj(5)*(N_ice/1000) + param.c_inj(6)*(N_ice/1000).^2;

eta_cal =  c_cal_2.*( u_f/100 - c_cal_1 ).^2 + param.c_inj(1);

% Indicated efficiency
eta_otto = 1 - 1./( param.r_c.^(gamma_cyl - 1));
eta_ig = eta_cal.*eta_otto;

end
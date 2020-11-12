function eta_t = turbineEfficiency(BSR, N_tc_corr, ice_param)
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
N_tc_norm = N_tc_corr.*1e-4;
eta_t_max = ice_param.turb.c_eta0 + ice_param.turb.c_eta1.*N_tc_norm + ice_param.turb.c_eta2.*N_tc_norm.^2;
k_eta     = ice_param.turb.c_max0 + ice_param.turb.c_max1.*N_tc_norm;
BSR_opt   = ice_param.turb.c_BSR0 + ice_param.turb.c_BSR1.*N_tc_norm + ice_param.turb.c_BSR2.*N_tc_norm.^2;
eta_t     = eta_t_max + k_eta.*(BSR-BSR_opt).^2;

end

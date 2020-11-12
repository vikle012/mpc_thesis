function [W_t, P_t_eta_tm, eta_tm, T_t] = turbine(w_tc, p_em, p_ats, T_em, param)
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
N_t_corr   = correctedTurbineSpeed(w_tc, T_em);
Pi_t       = turbinePressureRatio(p_ats, p_em); % [-]
BSR        = turbineBSR(Pi_t, w_tc, T_em, param);
W_t_corr   = correctedTurbineMassflow(Pi_t, N_t_corr, param); % [kg/s*kPa*K^-0.5]
W_t        = decorrectTurbineMassflow(W_t_corr, p_em, T_em);         % [kg/s]
eta_tm     = turbineEfficiency(BSR, N_t_corr, param);% [-]
P_t_eta_tm = turbinePower(W_t, T_em, Pi_t, eta_tm, param); % [W]
T_t        = turbineOutletTemperature(T_em, eta_tm, Pi_t, param); % [K]

end

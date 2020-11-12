function [ M_ice, signals ] = engineTorque(N_ice, p_im, p_em, u_f, phi, param)
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
%% Indicated work
[eta_ig, eta_otto, gamma, eta_cal] = indicatedEfficiency(phi, N_ice, u_f, param);
W_ig = param.q_HV .* eta_ig .* u_f * 1e-6 * param.n_cyl;
IMEP = W_ig./param.V_D;

%% Friction work
FMEP = frictionMeanEffectivePressure(N_ice, u_f, param);
W_fric = param.V_D.*FMEP;

%% Pump work
PMEP = pumpMeanEffectivePressure(p_em, p_im, param);
W_pump = param.V_D .* PMEP;

%% Engine Toruqe
M_ice = (W_ig - W_pump - W_fric)./(4*pi);
BMEP = M_ice*2*pi.*param.n_r./param.V_D;

%% Submodel signals
signals = struct;
signals.eta_ig = eta_ig;
signals.eta_otto = eta_otto;
signals.gamma = gamma;
signals.eta_cal = eta_cal;
signals.W_pump = W_pump;
signals.W_ig = W_ig;
signals.W_fric = W_fric;
signals.M_pump = W_pump/4/pi;
signals.M_ig = W_ig/4/pi;
signals.M_fric = W_fric/4/pi;
signals.BMEP = BMEP;
signals.IMEP = IMEP;
signals.PMEP = PMEP;
signals.FMEP = FMEP;

end


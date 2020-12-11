% function [W_out, Psi, Pi] = W_throttle(pus, pds, T_compressor, u_throttle, ice_param)
%
% Pi = pds./pus;
% Cd = ice_param.throttle.Cd;
% A = ice_param.throttle.A;
% Psi = ice_param.throttle.c_thr(1)*sqrt( 1 - Pi.^ice_param.throttle.c_thr(2) );
%
% W_out = pus./sqrt(ice_param.R_a.*T_compressor) * Cd .* A .* u_throttle .* Psi;
% end

function [W_thr, Psi, Pi] = W_throttle(pus, pds, T_c, u_thr, param)
Pi = pds./pus;

c3 = 4.548945000000000;
c2 = 0.999500000000000;
c1 = 0.756040668514975;
c4 = -0.036040668514975;
Psi = c1.*sqrt( 1 - (Pi.*c2).^c3 ) + c4;

W_thr = pus./sqrt(param.R_a.*T_c) .* param.throttle.Cd .* param.throttle.A .* u_thr .* Psi;
end

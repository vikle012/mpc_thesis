function W_out = W_wastegate( pus, pds, T_em, u_wg, ice_param )

Pi = pds./pus;
Psi = ice_param.c_wg(1)*sqrt( 1 - Pi.^ice_param.c_wg(2) );

W_out = pus./sqrt(ice_param.R_e.*T_em) * ice_param.A_wg .* u_wg .* Psi;

end
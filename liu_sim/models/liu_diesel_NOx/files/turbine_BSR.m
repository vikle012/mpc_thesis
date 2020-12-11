function BSR = turbine_BSR(PiT, w_t, T_em, param)

BSR = param.turb.R_t*w_t./sqrt( 2*param.c_pe*T_em.*(1 - PiT.^(1-1/param.gamma_cyl)) );

end
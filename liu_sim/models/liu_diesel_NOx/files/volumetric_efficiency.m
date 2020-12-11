function eta_vol = volumetric_efficiency(p_im, Ne, param)

eta_vol = param.c_vol(1)*sqrt(p_im) + ...
          param.c_vol(2)*sqrt(Ne) + ...
          param.c_vol(3);
end


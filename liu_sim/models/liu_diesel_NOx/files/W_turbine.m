function W_t_corr = W_turbine(PiT, Nt, param)

Nt = Nt./1e5;

k1 = param.turb.k1;

c10 = param.turb.c_param(1);
c11 = param.turb.c_param(2);
c12 = param.turb.c_param(3);

c20 = param.turb.c_param(4);
c21 = param.turb.c_param(5);
c22 = param.turb.c_param(6);

PiT_0 = 0;%c10 + c11.*(Nt).^c12;
k_0 = c20+c21.*Nt+ c22.*Nt.^2;


W_t_corr = k_0.*sqrt(1-(PiT-PiT_0).^k1);
 
end
function W_t_corr = turbine_massflow(PiT, Nt, param)

Nt = Nt./1e5;

k1 = param.turb.k1;

c10 = param.turb.c_param(1);
c11 = param.turb.c_param(2);
c12 = param.turb.c_param(3);

c20 = param.turb.c_param(4);
c21 = param.turb.c_param(5);
c22 = param.turb.c_param(6);

% PiT_0 = @(N) c10 + c11.*(N).^c12;
% k_0 = @(N) c20+c21.*Nt+ c22.*Nt.^2;

PiT_0 = c10 + c11.*(Nt).^c12;
k_0 = c20+c21.*Nt+ c22.*Nt.^2;
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % |x| = sqrt(x^2) ~ sqrt(x^2 + e), e > 0 
% cont_abs = @(x, e) sqrt(x.^2 + e);
% % max(a, b) = ( a + b + |a - b|)/2;
% cont_max = @(x1, x2, e) (x1 + x2 + cont_abs(x1-x2, e))/2;
% PiT_0 = cont_max(0, PiT_0, 0.0001);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
W_t_corr = k_0.*sqrt(1-(PiT-PiT_0).^k1);
% if ~isreal(W_t_corr)
% end
end
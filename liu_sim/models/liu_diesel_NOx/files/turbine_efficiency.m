function eta_tm = turbine_efficiency(BSR, TSP, param)

eta_tm_max = @(k,x,m) k.*x.^2 + m;
cm = @(cm1,cm2,cm3,omega) cm1.*(omega-cm2).^cm3;
BSR_opt = @(k,x,m,expn) k.*x.^expn + m;


eta_tm = eta_tm_max(param.turb.eta_tmmax_k,TSP./1e5,param.turb.eta_tmmax) - cm(param.turb.c_m1, param.turb.c_m2, param.turb.c_m3.*1e-3, TSP).*(BSR - BSR_opt(param.turb.BSR_opt_k,TSP./1e5,param.turb.BSR_opt,param.turb.BSR_opt_expn)).^2; 
end


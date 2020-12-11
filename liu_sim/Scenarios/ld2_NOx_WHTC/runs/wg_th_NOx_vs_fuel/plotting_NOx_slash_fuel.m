%% PLOTTING SCRIPT FOR SPECIFIC CASE

figure();
hold on;
title('NOx vs. Fuel');
ylabel('Normalized values');
xlabel('Time (s)');

%%
load('ld2_NOx_0_45_K1d5_with_fuel.mat');
scale_NOx = max(opt_signals.NOx_tot.val);

W_f(1) = opt_signals.W_f.val(1);
for i = 2:length(opt_signals.t_vec.val)
   W_f(i) = W_f(i-1) + opt_signals.W_f.val(i); 
end
scale_fuel = W_f(end);

plot(opt_signals.t_vec.val, opt_signals.NOx_tot.val./scale_NOx, 'r-');
plot(opt_signals.t_vec.val, W_f ./ scale_fuel, 'r--');

%%
% load('ld2_NOx_0_45_K1d5_combined_09_01.mat');
% 
% W_f(1) = opt_signals.W_f.val(1);
% for i = 2:length(opt_signals.t_vec.val)
%    W_f(i) = W_f(i-1) + opt_signals.W_f.val(i); 
% end
% 
% plot(opt_signals.t_vec.val, opt_signals.NOx_tot.val./scale_NOx)
% plot(opt_signals.t_vec.val, W_f ./ scale_fuel);

%%
% load('ld2_NOx_0_45_K1d5_combined_1_1.mat');
% 
% W_f(1) = opt_signals.W_f.val(1);
% for i = 2:length(opt_signals.t_vec.val)
%    W_f(i) = W_f(i-1) + opt_signals.W_f.val(i); 
% end
% 
% plot(opt_signals.t_vec.val, opt_signals.NOx_tot.val./scale_NOx)
% plot(opt_signals.t_vec.val, W_f ./ scale_fuel);

%%
load('ld2_NOx_0_45_K1d5_with_nox.mat');

W_f(1) = opt_signals.W_f.val(1);
for i = 2:length(opt_signals.t_vec.val)
   W_f(i) = W_f(i-1) + opt_signals.W_f.val(i); 
end

plot(opt_signals.t_vec.val, opt_signals.NOx_tot.val./scale_NOx, 'b-')
plot(opt_signals.t_vec.val, W_f ./ scale_fuel, 'b--');

l = line([0 45],[1 1]);
l.Color = [0.7 0.7 0.7];
legend('NO_x for fuel minimization',' Fuel for  for fuel minimization', ...
    'NO_x for NO_x minimization (-11.32%)','Fuel for NO_x minimization (+0.62%)', ...
    'Location', 'NorthWest');
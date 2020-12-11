
hev = @(x, u, t) PT_PWL__hev_eats_ss(x, u, t, param, t0);

t_vec = v0.T;
x_vec = v0.X;
u_vec = v0.U;

constraints = zeros(23,length(t_vec));
violation = zeros(length(t_vec),1);
k_prev = 0;

for k = 1:length(t_vec)
    
    [dX, constraints(:,k), signals] = hev(x_vec(k,:)', u_vec(k,:)', t_vec(k));
    if any(constraints(:,k) > 0)
        violation(k) = find(constraints(:,k) > 0, 1, 'first');
        fprintf('Violation at time: %.2f\n', t_vec(k));
    else
        violation(k) = 0;
    end
    if k - k_prev >= 1000
        fprintf('Current step: %.0f of %.0f \n', k, length(t_vec));
        k_prev = k;
    end
    
end
if ~isempty(violation(violation > 0))
    w = warndlg('Found constraint violation in initial guess. Continue?','Bad initial guess');
    waitfor(w);
end
%fprintf('Occurances of constraint violation: %.0f \n', nnz(violation));



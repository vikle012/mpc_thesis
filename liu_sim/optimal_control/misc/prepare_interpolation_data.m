function [s_ip, u_ip] = prepare_interpolation_data(t_opt, u_opt)

% Make control signal piecewise constant
t_opt(end+1) = t_opt(end)*10;
u_opt(end+1,:) = u_opt(end,:);

for k=1:length(t_opt)-1
    s_ip(2*k-1) = t_opt(k);
    s_ip(2*k) = t_opt(k+1)-0.001;
    u_ip(2*k-1,:) = u_opt(k,:);
    u_ip(2*k,:) = u_opt(k,:);
end

end
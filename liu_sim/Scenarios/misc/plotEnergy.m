function plotEnergy(sig1, sig2)

if ~exist('param', 'var')
    load('ld2_eng_map.mat', 'engine_map')
    regen_brake = 1000; % Nm
    param = createParam__hev_eats_tipin_whtc(engine_map, regen_brake);
end


%% SIGNAL 1
if sig1.x_vec.val(end,7) < 1
    % SIG1 is a hybrid
    
    E1 = zeros(length(sig1.t_vec.val),1);
    for i = 2:length(sig1.t_vec.val)
        E1(i) = trapz(sig1.t_vec.val(1:i), sig1.W_f.val(1:i)).*param.ld2.q_HV - ...
            (sig1.x_vec.val(i,7)-sig1.x_vec.val(1,7)).*param.batpack.Q;
    end
else
    % SIG1 is a conventional
    
    E1 = zeros(length(sig1.t_vec.val),1);
    for i = 2:length(sig1.t_vec.val)
        E1(i) = trapz(sig1.t_vec.val(1:i), sig1.W_f.val(1:i)).*param.ld2.q_HV;
    end
end

%% SIGNAL 2
if sig2.x_vec.val(end,7) < 1
    % SIG2 is a hybrid
    
    E2 = zeros(length(sig2.t_vec.val),1);
    for i = 2:length(sig2.t_vec.val)
        E2(i) = trapz(sig2.t_vec.val(1:i), sig2.W_f.val(1:i)).*param.ld2.q_HV - ...
            (sig2.x_vec.val(i,7)-sig2.x_vec.val(1,7)).*param.batpack.Q;
    end
else
    % SIG2 is a conventional
    
    E2 = zeros(length(sig2.t_vec.val),1);
    for i = 2:length(sig2.t_vec.val)
        E2(i) = trapz(sig2.t_vec.val(1:i), sig2.W_f.val(1:i)).*param.ld2.q_HV;
    end
end

%% PLOT
figure(); hold on;
title('Energy used');
plot(sig1.t_vec.val, E1, 'k');
plot(sig2.t_vec.val, E2, 'r--');

end


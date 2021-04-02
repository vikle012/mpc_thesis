%% Run after run_controlledSystem

M_e_eval = squeeze(simOut.simM_e);
W_f_eval = squeeze(simOut.simW_f);

M_desired = M_e_input;

for i = 1:length(M_e_input)
    
    % Closed rack motoring
    if M_e_input(i) < 0
        M_desired(i) = 0;
        M_e_eval(i) = 0;
        W_f_eval(i) = 0;
    end
    
end

T_cycle = simOut.simTime(end) - simOut.simTime(1);

e_Me = trapz(simOut.simTime, abs(M_desired - M_e_eval))/T_cycle

m_f = trapz(simOut.simTime, W_f_eval)

E_m_J = trapz(simOut.simTime, M_e_eval.*simOut.simn_e)*30/pi; % In Joule
E_m = E_m_J*2.77777778e-7 % in kWh

% g/kWh
m_f*1000/E_m
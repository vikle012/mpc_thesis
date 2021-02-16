% Run with simulation data accessible

T_cycle = simOut.simTime(end) - simOut.simTime(1);

eMe = trapz(simOut.simTime, abs(M_e_input - squeeze(simOut.simM_e)))/T_cycle

mf = trapz(simOut.simTime, squeeze(simOut.simW_f))

% In Joule
Em_J = trapz(simOut.simTime, squeeze(simOut.simM_e).*simOut.simn_e)*30/pi;
Em = Em_J*2.77777778e-7 % kWh


%%

% % Memory issues using this
% idx = find(M_e_input < 0);
% 
% % Total fuel
% trapz(simulate.T_s, squeeze(simOut.simW_f))
% 
% % Tracking error
% trapz(simulate.T_s, abs(M_e_input - squeeze(simOut.simM_e)))/1799
% 
% % Total work
% trapz(simulate.T_s, squeeze(simOut.simM_e).*simOut.simn_e)*30/pi
% 
% % Testing total work
% Me = squeeze(simOut.simM_e);
% Me_new = [];
% ne_new = [];
% for i = 1:length(Me)
%    
%     if Me(i) >= 0
%        Me_new = [Me_new; Me(i)];
%        ne_new = [ne_new; simOut.simn_e(i)];
%     end   
% end
% trapz(simulate.T_s, Me_new.*ne_new)*30/pi

% replace all neg. in desired and Me with 0?

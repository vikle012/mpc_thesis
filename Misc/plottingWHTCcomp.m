clear
load('LMPC')

simOutLMPC = simOut;

load('SLref_WHTC')

close all

h3 = figure(3);
hold on
plot(time_vector, M_e_input, 'k:');
plot(simOutLMPC.simTime, squeeze(simOutLMPC.simM_e))
plot(simOut.simTime, squeeze(simOut.simM_e), '--')
legend(["Reference", "LMPC", "SLMPC: reference"], 'Location', 'southeast')
%ylim([-200 2800])
ylabel('M_e [Nm]')
xlabel('Time [s]')

% set(h3,'Units','Inches');
% pos = get(h3,'Position');
% set(h3,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
% print(h3, 'Figures/Results/MPC_WHTC_compare','-dpdf','-r0')


h4 = figure(4);
hold on
plot(time_vector, M_e_input, 'k:');
plot(simOutLMPC.simTime, squeeze(simOutLMPC.simM_e))
plot(simOut.simTime, squeeze(simOut.simM_e), '--')
%legend(["Reference", "LMPC", "SLMPC: reference"], 'Location', 'southeast')
%ylim([-200 2800])
xlim([1290 1460])
ylabel('M_e [Nm]')
xlabel('Time [s]')

set(h4,'Units','Inches');
pos = get(h4,'Position');
set(h4,'PaperPositionMode','Auto','PaperUnits','Inches','PaperSize',[pos(3), pos(4)])
print(h4, 'Figures/Results/MPC_WHTC_compare_zoom','-dpdf','-r0')
function [dXdt, signals, parameters] = EATS_model(X, V, U, parameters, signals)

%==========================================================================
% Parameters
%==========================================================================
DOC = parameters.DOC;
DPF = parameters.DPF;
UDP = parameters.UDP;
SCR = parameters.SCR;
EXH_GAS = parameters.EXH_GAS;

%==========================================================================
% States
%==========================================================================
% X = [ T_DOC; (#DOC-segment states) 
%       T_DPF; (#DPF-segment states)
%       T_SCR; (#SCR-segment states)


n = 1;
T_DOC = X(n:n+DOC.n_seg-1);
n = n + DOC.n_seg;
T_DPF = X(n:n+DPF.n_seg-1);
n = n + DPF.n_seg;
T_SCR = X(n:n+SCR.n_seg-1);
n = n + SCR.n_seg;
T_silencer = X(n:n+1);


%==========================================================================
% Exogenous inputs
%==========================================================================
% V = [ W;
%       p;
%       T_bDoc;
%       T_amb];

W = V(1);
p = V(2);
T_bDOC = V(3);
T_amb = V(4);

%==========================================================================
% Control signals
%==========================================================================
% U = u_urea
% u_urea = U;

%==========================================================================
% Silencer
%==========================================================================

dT_silencer_dt = silencer_model(T_silencer, T_DOC, T_DPF, T_SCR, T_amb, parameters);

T_shell = T_silencer(2);
T_in = T_silencer(1);
T_inside = T_silencer(1);

%==========================================================================
% DOC
%==========================================================================
[dT_DOCdt, T_aDOC] = substrate_thermal_model(T_DOC, W/parameters.N_docs, T_bDOC, T_inside, p,...
    DOC, EXH_GAS);

T_bDPF = T_aDOC;

%==========================================================================
% DPF
%==========================================================================
[dT_DPFdt, T_aDPF] = substrate_thermal_model(T_DPF, W/parameters.N_dpfs, T_bDPF, T_inside, p,...
    DPF, EXH_GAS);

T_bUDP = T_aDPF;

%==========================================================================
% UDP
%==========================================================================

T_bSCR = T_bUDP;

%==========================================================================
% SCR
%==========================================================================
[dT_SCRdt, T_aSCR] = substrate_thermal_model(T_SCR, W/parameters.N_scrs, T_bSCR, T_inside, p,...
    SCR, EXH_GAS);

%==========================================================================
% Derivatives
%==========================================================================
dXdt = [dT_DOCdt; dT_DPFdt; dT_SCRdt; dT_silencer_dt];% dth_NH3dt];

%==========================================================================
% deNOx performance
%==========================================================================
T_min = 200;
[deNOx_Cu, deNOx_Fe, NOx_TP_Cu, NOx_TP_Fe] = deNOx_fun(T_SCR(3), signals);

%==========================================================================
% Output signals
%==========================================================================

% if nargout > 1
%     if isempty(signals)
%         signals.T_aDOC = T_aDOC;
%         signals.T_aDPF = T_aDPF;
%         signals.T_aSCR = T_aSCR;
%         signals.T_inside = T_in;
%         signals.T_shell = T_shell;
%     else
%         signals.T_aDOC(end+1,:) = T_aDOC;
%         signals.T_aDPF(end+1,:) = T_aDPF;
%         signals.T_aSCR(end+1,:) = T_aSCR;
%         signals.T_inside(end+1,:) = T_in;
%         signals.T_shell(end+1,:) = T_shell;
%     end

signals=struct(...
    'T_aDOC'  , T_aDOC - 273, ... To celsius
    'T_aDPF'  , T_aDPF - 273, ... To celsius
    'T_aSCR'  , T_aSCR - 273, ... To celsius
    'deNOx_Cu'   , deNOx_Cu, ...
    'deNOx_Fe'   , deNOx_Fe, ...
    'NOx_TP_Cu'  , NOx_TP_Cu, ...
    'NOx_TP_Fe'  , NOx_TP_Fe, ...
    'T_SCR1'  , T_SCR(1), ...
    'T_SCR2'  , T_SCR(2), ...
    'T_SCR3'  , T_SCR(3), ...
    'T_SCR4'  , T_SCR(4), ...
    'T_SCR5'  , T_SCR(5) ...
    );
end
    
  
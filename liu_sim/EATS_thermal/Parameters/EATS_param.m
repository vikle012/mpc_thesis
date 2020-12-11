%==============================================================================
% EATS
%==============================================================================
EATS = [];
EATS.N_docs = 1;
EATS.N_dpfs = 2;
EATS.N_scrs = 3;

%==============================================================================
% EXH_GAS
%==============================================================================
EXH_GAS.c_p= 1250;
EXH_GAS.R = 241.121;

EATS.EXH_GAS = EXH_GAS;
clear EXH_GAS;
%==============================================================================
% Silencer
%==============================================================================
Silencer.Geometry.A_shell = 1.44;

Silencer.Heat_transfer.h_solidAmbient = 15;
Silencer.Heat_transfer.h_solidAir = 10;

Silencer.m_air = 0.24;
Silencer.m_shell = 11.232;
Silencer.c_p_shell = 466;
Silencer.c_p_inside = 1050;

EATS.Silencer = Silencer;
clear Silencer;
%==============================================================================
% DOC
%==============================================================================

DOC.n_seg = 2;
DOC.Geometry.V = 0.0071804;
DOC.Geometry.D = 10.5*2.54e-2*sqrt(2); 
DOC.Geometry.L = 4.5*2.54e-2*2; 
DOC.Geometry.a_solidambient = 0.0875;
DOC.Geometry.epsilon = 0.8343;

DOC.Heat_transfer.h_solidambient = 250;%350000/8.75;

DOC.Solid.c_p = 1050;
DOC.Solid.rho = 3100*1.5;
DOC.Solid.c_p = 4184*0.35;
DOC.Solid.rho = 2600*1.5;

EATS.DOC = DOC;
clear DOC;
%==============================================================================
% DPF
%==============================================================================
DPF.n_seg = 3;

DPF.Solid.c_p = 1050;
DPF.Solid.rho = 2000;
DPF.Solid.c_p = 4184*0.35*.7;
DPF.Solid.rho = 2600;

DPF.Geometry.epsilon = 0.7722;
DPF.Geometry.D = 10.5*2.54e-2;%/sqrt(1.5); 
DPF.Geometry.a_solidambient = 0.2;
DPF.Geometry.L = 9*2.54e-2;%/1.5; 

DPF.Heat_transfer.h_solidambient = 250;%25;

EATS.DPF = DPF;
clear DPF;
%==============================================================================
% UDP
%==============================================================================

UDP.Geometry.L_post = 1;
UDP.Geometry.D_post = 0.127;

EATS.UDP = UDP;
clear UDP;
%==============================================================================
% SCR
%==============================================================================
SCR.n_seg = 5;

SCR.Geometry.D = 11.25*2.54e-2;%/sqrt(1.5); 
SCR.Geometry.epsilon = 0.6327;
SCR.Geometry.L = (8+3)*2.54e-2;%1.5;
SCR.Geometry.a_solidambient = 0.89;

SCR.Solid.c_p = 4184*0.35;
SCR.Solid.rho = 2600;

SCR.Heat_transfer.h_solidambient = 250;

EATS.SCR = SCR;
clear SCR;
%==============================================================================
% Initial values
%==============================================================================

EATS.T_DOC_0 = ones(EATS.DOC.n_seg,1)*(294);
EATS.T_DPF_0 = ones(EATS.DPF.n_seg,1)*(294);
EATS.T_SCR_0 = ones(EATS.SCR.n_seg,1)*(294);
EATS.T_silencer_0 = [294 294]';

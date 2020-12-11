function z = internalSignals__hev_eats_whtc(hev_eats_whtc, t_vec, x_vec, u_vec, t_label)

z = struct;

[~, ~, ~, parameters_used] = hev_eats_whtc(x_vec(1,:)', u_vec(1,:)', t_vec(1));

z.t_vec.val = t_vec;
z.x_vec.val = x_vec;
z.x_vec.title = 'States';
z.x_vec.label = 'x';
z.u_vec.val = u_vec;
z.u_vec.title = 'Control inputs';
z.u_vec.label = 'u';

if nargin == 5
    z.t_vec.label = t_label;
else
    z.t_vec.label = 'Time [s]';
end

for k = 1:length(t_vec)
    
    [~, ~, c] = hev_eats_whtc(x_vec(k,:)', u_vec(k,:)', t_vec(k));
    
    % CAC and Throttle
    z.C1 = 'CAC and Throttle'; % Category (for readability)
    z.p_cac.val(k)  = c.p_cac;
    z.W_thr.val(k)  = c.W_thr;
    z.u_thr.val(k)  = c.u_thr;
    z.Pi_thr.val(k) = c.Pi_thr;
    % Intake manifold
    z.C2 = 'Intake Manifold';
    z.p_im.val(k)   = c.p_im;
    z.W_cyl.val(k)  = c.W_cyl;
    z.eta_vol.val(k)= c.eta_vol;
    % Cylinder
    z.C3 = 'Cylinder';
    z.N_ice.val(k)  = c.N_ice;
    z.M_ice.val(k)  = c.M_ice;
    z.P_ice.val(k)  = c.P_ice;
    z.u_f.val(k)    = c.u_f;
    z.W_f.val(k)    = c.W_f;
    z.P_f.val(k)    = c.P_f;
    z.Lambda.val(k) = c.Lambda;
    z.phi.val(k)    = c.phi;
    z.eta_f.val(k)  = c.eta_f;
    z.M_fric.val(k) = c.M_fric;
    z.M_pump.val(k) = c.M_pump;
    z.M_ig.val(k)   = c.M_ig;
    % Exhaust manifold
    z.C4 = 'Exhaust Manifold';
    z.p_em.val(k)   = c.p_em;
    z.T_em.val(k)   = c.T_em;
    % Turbo
    z.C5 = 'Turbocharger';
    z.w_t.val(k)    = c.w_t;
    z.N_t.val(k)    = c.N_t;
    z.eta_tm.val(k) = c.eta_tm;
    % Turbine
    z.C5a = 'Turbine';
    z.Pi_turb.val(k) = c.Pi_turb;
    z.turb_exp_rat.val(k) = 1/c.Pi_turb;
    z.W_wg.val(k)   = c.W_wg;
    z.u_wg.val(k)   = c.u_wg;
    z.W_turb.val(k) = c.W_turb;
    z.BSR.val(k)    = c.BSR;
    z.P_turb_eta_tm.val(k) ...
        = c.P_turb_eta_tm;
    z.T_turb.val(k) = c.T_turb;
    z.T_turb_drop.val(k) = c.T_turb_drop;
    % Compressor
    z.C5b = 'Compressor';
    z.Pi_comp.val(k) = c.Pi_comp;
    z.W_comp.val(k) = c.W_comp;
    z.eta_comp.val(k) = c.eta_comp;
    z.T_comp.val(k)  = c.T_comp;
    z.P_comp.val(k)  = c.P_comp;
    % After Turbine and Wastegate
    z.C5c = 'After Turbine and Wastegate';
    z.T_atw.val(k)      = c.T_atw;
    z.T_atw_c.val(k)    = c.T_atw - 273; % To celcius
    z.T_tw_drop.val(k)  = c.T_tw_drop;
    z.W_tw.val(k)       = c.W_tw;
    % Emissions
    z.C6 = 'Emissions';
    z.NOx_EO.val(k)    = c.NOx_EO;
    z.NOx_tot.val(k)= c.NOx_tot;
    % Aftertreatment system
    z.C7 = 'Aftertreatment system';
    z.T_aDOC.val(k) = c.T_aDOC;
    z.T_aDPF.val(k) = c.T_aDPF;
    z.T_aSCR.val(k) = c.T_aSCR;
    z.T_SCR1.val(k) = c.T_SCR1;
    z.T_SCR2.val(k) = c.T_SCR2;
    z.T_SCR3.val(k) = c.T_SCR3;
    z.T_SCR4.val(k) = c.T_SCR4;
    z.T_SCR5.val(k) = c.T_SCR5;
    z.deNOx_Cu.val(k)  = c.deNOx_Cu;
    z.NOx_TP_Cu.val(k) = c.NOx_TP_Cu;
    z.deNOx_Fe.val(k)  = c.deNOx_Fe;
    z.NOx_TP_Fe.val(k) = c.NOx_TP_Fe;
    % ELECTRIC POWERTRAIN
    z.C8 = 'Electric Powertrain';
    z.P_em_mech.val(k)  = c.P_em_mech;
    z.eta_em.val(k)     = c.eta_em;
    z.P_em_elec.val(k)  = c.P_em_elec;
    z.U_oc.val(k)       = c.U_oc;
    z.U_b.val(k)        = c.U_b;
    z.I_b.val(k)        = c.I_b;
    z.M_em.val(k)       = c.M_em;
    z.eta_b.val(k)      = c.eta_b;
    z.SOC.val(k)        = c.SOC;
    z.P_b.val(k)        = c.P_b;
    %HYBRID SIGNALS
    z.C9 = 'Hybrid signals';
    z.M_ratio.val(k)    = c.M_ratio;
    z.M_em_norm.val(k)  = c.M_em_norm;
    z.M_ice_norm.val(k) = c.M_ice_norm;
    z.M_tot.val(k)      = c.M_tot;
end

% CAC and Throttle
z.p_cac.title = 'Charge air cooler';
z.p_cac.label = 'p_{cac} [Pa]';
z.W_thr.title = 'Throttle massflow';
z.W_thr.label = 'W_{thr} [kg/s]';
z.u_thr.title = 'Throttle control input';
z.u_thr.label = 'u_{thr} [0, 1]';
z.Pi_thr.title = 'Throttle pressure ratio';
z.Pi_thr.label = '\Pi_{thr} [-]';
% Intake manifold
z.p_im.title = 'Intake manifold pressure';
z.p_im.label = 'p_{im} [Pa]';
z.W_cyl.title = 'Cylinder air massflow';
z.W_cyl.label = 'W_{cyl} [kg/s]';
z.eta_vol.title = 'volumetric efficiency';
z.eta_vol.label = '\eta_{vol} [-]';
% Cylinder
z.N_ice.title = 'Engine speed';
z.N_ice.label = 'N_{ice} [rpm]';
z.M_ice.title = 'Engine torque';
z.M_ice.label = 'M_{ice} [Nm]';
z.P_ice.title = 'Engine power';
z.P_ice.label = 'P_{ice} [W]';
z.u_f.title = 'Fuel injection';
z.u_f.label = 'u_f [mg/cyc]';
z.W_f.title = 'Fuel flow';
z.W_f.label = 'W_f [kg/s]';
z.P_f.title = 'Power in fuel';
z.P_f.label = 'P_f [J/s]';
z.Lambda.title = 'Air-to-fuel eq. ratio';
z.Lambda.label = '\lambda [-]';
z.phi.title = 'Fuel-to-air eq. ratio';
z.phi.label = '\phi [-]';
z.eta_f.title = 'Fuel efficiency';
z.eta_f.label = '\eta_f [-]';
z.M_fric.title = 'Friction torque';
z.M_fric.label = 'M_{fric} [Nm]';
z.M_pump.title = 'Pumping torque';
z.M_pump.label = 'M_{pump} [Nm]';
z.M_ig.title = 'Indicated torque';
z.M_ig.label = 'M_{ig} [Nm]';
% Exhaust manifold
z.p_em.title = 'Exhaust manifold pressure';
z.p_em.label = 'p_{em} [Pa]';
z.T_em.title = 'Exhaust manifold temperature';
z.T_em.label = 'T_{em} [K]';
% Turbocharger
z.w_t.title = 'Turbocharger speed';
z.w_t.label = '\omega_t [rad/s]';
z.N_t.title = 'Turbocharger speed';
z.N_t.label = 'N_t [krpm]';
z.eta_tm.title = 'Turbine and shaft efficiency';
z.eta_tm.label = '\eta_{tm} [-]';
z.Pi_turb.title = 'Turbine pressure ratio';
z.Pi_turb.label = '\Pi_t [-]';
z.turb_exp_rat.title = 'Turbine expanssion ratio';
z.turb_exp_rat.label = '1/\Pi_t [-]';
z.W_wg.title = 'Wastegate massflow';
z.W_wg.label = 'W_{wg} [kg/s]';
z.u_wg.title = 'Wastegate control input';
z.u_wg.label = 'u_{wg} [0, 1]';
z.W_turb.title = 'Turbine massflow';
z.W_turb.label = 'W_{t} [kg/s]';
z.BSR.title = 'BSR';
z.BSR.label = 'BSR [-]';
z.P_turb_eta_tm.title = 'Turbine power';
z.P_turb_eta_tm.label = 'P_{t}\eta_{tm} [W]';
z.T_turb.title = 'Temp. after turbine';
z.T_turb.label = 'Turbine temperature [K]';
z.T_turb_drop.title = 'Temperature drop over turbine';
z.T_turb_drop.label = 'Temperature drop over turbine [K]';
z.Pi_comp.title = 'Compressor pressure ratio';
z.Pi_comp.label = '\Pi_c [-]';
z.W_comp.title = 'Compressor massflow';
z.W_comp.label = 'W_c [kg/s]';
z.eta_comp.title = 'Compressor efficiency';
z.eta_comp.label = '\eta_c [-]';
z.T_comp.title  = 'Compressor compressed air temp.';
z.T_comp.label  = 'T_c [K]';
z.P_comp.title  = 'Compressor power';
z.P_comp.label  = 'P_c [W]';
% After turbine and wastegate
z.T_atw.title = 'Temperature after Turbine and Wastegate';
z.T_atw.label = 'T_{atw} [K]';
z.T_atw_c.title = 'Temperature after Turbine and Wastegate';
z.T_atw_c.label = 'T_{atw} [°C]';
z.T_tw_drop.title = 'Temperature drop over Turbine and Wastegate';
z.T_tw_drop.label = '\Delta T_{tw} [K]';
z.W_tw.title = 'Massflow past Turbine and Wastegate';
z.W_tw.label = 'W_{tw} [kg/s]';
% Emissions
z.NOx_EO.title  = 'Out of engine NO_{x}';
z.NOx_EO.label  = 'NO_{x} [g/s]';
z.NOx_tot.title = 'Out of engine total NO_{x}';
z.NOx_tot.label = 'NO_{x} [g]';
% Aftertreatment system
z.T_aDOC.title = 'Diesel oxidation catalyst';
z.T_aDOC.label = 'Temperature [°C]';
z.T_aDPF.title = 'Diesel particulate filter';
z.T_aDPF.label = 'Temperature [°C]';
z.T_aSCR.title = 'SCR catalyst';
z.T_aSCR.label = 'Temperature [°C]';
z.T_SCR1.title    = 'SCR1';
z.T_SCR1.label    = 'Temperature [K]';
z.T_SCR2.title    = 'SCR2';
z.T_SCR2.label    = 'Temperature [K]';
z.T_SCR3.title    = 'SCR3';
z.T_SCR3.label    = 'Temperature [K]';
z.T_SCR4.title    = 'SCR4';
z.T_SCR4.label    = 'Temperature [K]';
z.T_SCR5.title    = 'SCR5';
z.T_SCR5.label    = 'Temperature [K]';
z.deNOx_Cu.title  = 'CuZ catalyst NOx conversion';
z.deNOx_Cu.label  = 'NOx conversion [-]';
z.NOx_TP_Cu.title  = 'CuZ catalyst Tailpipe NOx';
z.NOx_TP_Cu.label  = 'NOx [kg]';
z.deNOx_Fe.title  = 'FeZ catalyst NOx conversion';
z.deNOx_Fe.label  = 'NOx conversion [-]';
z.NOx_TP_Fe.title  = 'FeZ catalyst Tailpipe NOx';
z.NOx_TP_Fe.label  = 'NOx [kg]';
% ELECTRIC POWERTRAIN
z.P_em_mech.title = 'Electric motor mechanical power';
z.P_em_mech.label = 'P_{em, mech} [W]';
z.eta_em.title = 'Electric motor efficiency';
z.eta_em.label = '\eta_{em} [%]';
z.P_em_elec.title = 'Electric motor electric power';
z.P_em_elec.label = 'P_{em, elec} [W]';
z.U_oc.title = 'Open circuit voltage';
z.U_oc.label = 'U_{oc} [V]';
z.U_b.title = 'Battery voltage';
z.U_b.label = 'U_{b} [V]';
z.I_b.title = 'Battery current';
z.I_b.label = 'I_{b} [A]';
z.M_em.title = 'Electric motor torque';
z.M_em.label = 'M_{em} [Nm]';
z.eta_b.title = 'Battery efficiency';
z.eta_b.label = '\eta_{b} [%]';
z.SOC.title = 'State of charge';
z.SOC.label = 'SOC [%]';
z.P_b.title = 'Battery power';
z.P_b.label = 'Power [W]';
% HYBRID SIGNALS
z.M_ratio.title = 'EM:ICE torque ratio';
z.M_ratio.label = 'M_{em}/M_{ice}';
z.M_em_norm.title = 'Electric motor torque normalized to cycle demand';
z.M_em_norm.label = 'M_{em}/M_{demand}';
z.M_ice_norm.title = 'Combustion engine torque normalized to cycle demand';
z.M_ice_norm.label = 'M_{ice}/M_{demand}';
z.M_tot.title = 'Sum of EM and ICE torque';
z.M_tot.label = 'Torque [Nm]';


z.P = 'Parameters';
z.parameters = parameters_used;

end

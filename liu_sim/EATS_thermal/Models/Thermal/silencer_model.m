function dTdt = silencer_model(T_sil, T_doc, T_dpf, T_scr, T_amb, parameters)

T_doc = mean(T_doc);
T_dpf = mean(T_dpf);
T_scr = mean(T_scr);

T_inside = T_sil(1);
T_shell = T_sil(2);

Q_shell2inside = parameters.Silencer.Geometry.A_shell*parameters.Silencer.Heat_transfer.h_solidAir*(T_shell - T_inside);

Q_amd2shell = parameters.Silencer.Geometry.A_shell*parameters.Silencer.Heat_transfer.h_solidAmbient*(T_amb - T_shell);

Q_doc = parameters.DOC.Geometry.a_solidambient*parameters.DOC.Geometry.L...
    *parameters.Silencer.Heat_transfer.h_solidAir*(T_doc - T_inside);

Q_dpf = parameters.DPF.Geometry.a_solidambient*parameters.DPF.Geometry.L...
    *parameters.Silencer.Heat_transfer.h_solidAir*(T_dpf - T_inside);

Q_scr = parameters.SCR.Geometry.a_solidambient*parameters.SCR.Geometry.L...
    *parameters.Silencer.Heat_transfer.h_solidAir*(T_scr - T_inside);

dT_inside = (Q_shell2inside +  Q_doc + Q_dpf + Q_scr) / parameters.Silencer.c_p_inside/parameters.Silencer.m_air;

dT_shell = (Q_amd2shell - Q_shell2inside) / parameters.Silencer.c_p_shell/parameters.Silencer.m_shell;

dTdt = [dT_inside; dT_shell];
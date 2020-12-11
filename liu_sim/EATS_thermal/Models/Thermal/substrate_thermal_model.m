function [dTdt, T_out] = substrate_thermal_model(T, W, T_in, T_amb, p, substrate_parameters, gas_parameters)

    Geometry = substrate_parameters.Geometry;
    Solid = substrate_parameters.Solid;
    Heat_transfer = substrate_parameters.Heat_transfer;

    %rho = p/gas_parameters.R./T;
    A = Geometry.epsilon*pi*(Geometry.D^2)/4;
    
    v_times_rho = W/A;
   
    A_cross = (1-Geometry.epsilon)*pi*(Geometry.D^2)/4;
    h = 300000/2;
    
    %Q_cond = ([T(2:end); 0] + [0; T(1:end-1)] - [T(1:end-1); 0] - [0; T(2:end)])*A_cross*h./Geometry.L*substrate_parameters.n_seg^2;

    Q_amb = (T-T_amb)*Heat_transfer.h_solidambient*Geometry.a_solidambient;

    Q_gas_transport = v_times_rho*gas_parameters.c_p.*(T-[T_in;T(1:end-1)])./Geometry.L*substrate_parameters.n_seg;

    dTdt  = (-Q_gas_transport - Q_amb )/(1-Geometry.epsilon)/Solid.c_p/Solid.rho;
    
    T_out = T(end);
end
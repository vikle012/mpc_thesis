function [x_ref, u_ref] = reference_generator(M_e_ref, n_e_ref)
%REFERENCE_GENERATOR Summary of this function goes here
%   Detailed explanation goes here

load('table_data_relaxed')
load('max_torque_curve')

if length(M_e_ref) ~= length(n_e_ref)
    error('Unmatching dimensions')
end

% Normalize M_e_ref
M_e_max = interp1(N_e, M_e, n_e_ref);
M_e_norm = 100*M_e_ref./M_e_max;

% Generate lookup tables
x1_lut = casadi.interpolant('LUT','linear',{M_e_axis, n_e_axis},x1_table_data(:));
x2_lut = casadi.interpolant('LUT','linear',{M_e_axis, n_e_axis},x2_table_data(:));
x3_lut = casadi.interpolant('LUT','linear',{M_e_axis, n_e_axis},x3_table_data(:));
x4_lut = casadi.interpolant('LUT','linear',{M_e_axis, n_e_axis},x4_table_data(:));
x5_lut = casadi.interpolant('LUT','linear',{M_e_axis, n_e_axis},x5_table_data(:));

u1_lut = casadi.interpolant('LUT','linear',{M_e_axis, n_e_axis},u1_table_data(:));
u2_lut = casadi.interpolant('LUT','linear',{M_e_axis, n_e_axis},u2_table_data(:));
u3_lut = casadi.interpolant('LUT','linear',{M_e_axis, n_e_axis},u3_table_data(:));

% Generate references
x_ref = zeros(5, length(M_e_norm));
u_ref = zeros(3, length(M_e_norm));
for i = 1:length(M_e_norm)
    if M_e_norm(i) < 0
        x_ref(1,i) = -1;
        x_ref(2,i) = -1;
        x_ref(3,i) = -1;
        x_ref(4,i) = -1;
        x_ref(5,i) = -1;

        u_ref(1,i) = -1;
        u_ref(2,i) = -1;
        u_ref(3,i) = -1;
    else
        ref = [M_e_norm(i) n_e_ref(i)];

        x_ref(1,i) = full(x1_lut(ref));
        x_ref(2,i) = full(x2_lut(ref));
        x_ref(3,i) = full(x3_lut(ref));
        x_ref(4,i) = full(x4_lut(ref));
        x_ref(5,i) = full(x5_lut(ref));

        u_ref(1,i) = full(u1_lut(ref));
        u_ref(2,i) = full(u2_lut(ref));
        u_ref(3,i) = full(u3_lut(ref));
    end
end

end


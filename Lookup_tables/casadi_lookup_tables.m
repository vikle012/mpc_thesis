% Generate Lookup Tables
% Testing file

load('table_data_relaxed')

% [X,Y] = ndgrid(M_e_axis, n_e_axis);
x1_lut = casadi.interpolant('LUT','bspline',{M_e_axis, n_e_axis},x1_table_data(:));
x2_lut = casadi.interpolant('LUT','bspline',{M_e_axis, n_e_axis},x2_table_data(:));
x3_lut = casadi.interpolant('LUT','bspline',{M_e_axis, n_e_axis},x3_table_data(:));
x4_lut = casadi.interpolant('LUT','bspline',{M_e_axis, n_e_axis},x4_table_data(:));
x5_lut = casadi.interpolant('LUT','bspline',{M_e_axis, n_e_axis},x5_table_data(:));

u1_lut = casadi.interpolant('LUT','bspline',{M_e_axis, n_e_axis},u1_table_data(:));
u2_lut = casadi.interpolant('LUT','bspline',{M_e_axis, n_e_axis},u2_table_data(:));
u3_lut = casadi.interpolant('LUT','bspline',{M_e_axis, n_e_axis},u3_table_data(:));

% Testing 
% x1_lut([100 2000]) % [M_e n_e] 
% x1_lut([0 500])
% x1_lut([100, 5; 2000, 2000]) % Vector-wise


%% Illustaration that it works

newMe = 0:2:100;
newne = 500:10:2000;

newx1 = zeros(length(newMe), length(newne));
for i = 1:length(newne)
   for j = 1:length(newMe)
       newx1(j,i) = full(x1_lut([newMe(j) newne(i)]));
   end
end

surf(n_e_axis, M_e_axis, x1_table_data)
hold on
surf(newne, newMe, newx1)
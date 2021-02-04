%% Interpolate/extrapolate

% x1_table_data(find(~x1_table_data)) = nan;
x1_table_data_approx = inpaint_nans(x1_table_data, 1);
x2_table_data_approx = inpaint_nans(x2_table_data, 1);
x3_table_data_approx = inpaint_nans(x3_table_data, 1);
x4_table_data_approx = inpaint_nans(x4_table_data, 1);
x5_table_data_approx = inpaint_nans(x5_table_data, 1);
u1_table_data_approx = inpaint_nans(u1_table_data, 1);
u2_table_data_approx = inpaint_nans(u2_table_data, 1);
u3_table_data_approx = inpaint_nans(u3_table_data, 1);

%% Plotting

M_e_axis = 0:5:100;
n_e_axis = 500:25:2000;

figure
hold on
subplot(1,2,1)
surf(n_e_axis, M_e_axis, x1_table_data)
title('x_1')
subplot(1,2,2)
surf(n_e_axis, M_e_axis, x1_table_data_approx)
title('x_1 extrapolated')

figure
hold on
subplot(1,2,1)
surf(n_e_axis, M_e_axis, x2_table_data)
title('x_2')
subplot(1,2,2)
surf(n_e_axis, M_e_axis, x2_table_data_approx)
title('x_2 extrapolated')

figure
hold on
subplot(1,2,1)
surf(n_e_axis, M_e_axis, x3_table_data)
title('x_3')
subplot(1,2,2)
surf(n_e_axis, M_e_axis, x3_table_data_approx)
title('x_3 extrapolated')

figure
hold on
subplot(1,2,1)
surf(n_e_axis, M_e_axis, x4_table_data)
title('x_4')
subplot(1,2,2)
surf(n_e_axis, M_e_axis, x4_table_data_approx)
title('x_4 extrapolated')

figure
hold on
subplot(1,2,1)
surf(n_e_axis, M_e_axis, x5_table_data)
title('x_5')
subplot(1,2,2)
surf(n_e_axis, M_e_axis, x5_table_data_approx)
title('x_5 extrapolated')

figure
hold on
subplot(1,2,1)
surf(n_e_axis, M_e_axis, u1_table_data)
title('u_1')
subplot(1,2,2)
surf(n_e_axis, M_e_axis, u1_table_data_approx)
title('u_1 extrapolated')

figure
hold on
subplot(1,2,1)
surf(n_e_axis, M_e_axis, u2_table_data)
title('u_2')
subplot(1,2,2)
surf(n_e_axis, M_e_axis, u2_table_data_approx)
title('u_2 extrapolated')

figure
hold on
subplot(1,2,1)
surf(n_e_axis, M_e_axis, u3_table_data)
title('u_3')
subplot(1,2,2)
surf(n_e_axis, M_e_axis, u3_table_data_approx)
title('u_3 extrapolated')
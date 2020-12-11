function plot_calibration_subplot(subplot_no, M, WG_conv, WG_higheff, font_size, ylab, xlab, tit, ylim, xlim, location)

subplot(6,1,subplot_no); hold on; grid on;
plot(M, WG_conv, 'x-', 'LineWidth', 2)
plot(M, WG_higheff, 'x-', 'LineWidth', 2)
ylabel(ylab)
xlabel(xlab)
title(tit)
legend('Convetional', 'Efficiency optimized', 'Location', location)
plt = gca;
plt.YLim = ylim;
plt.XLim = xlim;
plt.FontSize = font_size;
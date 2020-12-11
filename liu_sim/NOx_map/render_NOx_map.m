function render_NOx_map(NOx, line_grid, line_text_size, axis_text_size, text_spacing, xaxis_text, yaxis_text, title_text)
X = NOx.wix;
Y = NOx.Tix;
Z = NOx.NOx;

[C, h] = contourf( ...
    X, ...
    Y, ...
    Z, ...
    line_grid, ...
    'ShowText','on', ...
    'LabelSpacing', text_spacing ...
    );
clabel(C, h, 'FontSize', line_text_size);
% colormap lines
plt = gca;
plt.FontSize = axis_text_size;

fill([NOx.wix(1,1) NOx.wix(end,:) NOx.wix(end,end)], [2400; NOx.Tmax; 2400], 'w');
plot(X(1,:), NOx.Tmax', '-', 'Color', [1 0.5 0], 'LineWidth', 2)
fill([NOx.wix(1,1) NOx.wix(end,:) NOx.wix(end,end)], [-263.3; NOx.Tmin; -263.3], 'w');
plot(X(1,:), NOx.Tmin', '-', 'Color', [1 0.5 0], 'LineWidth', 2)

%ylim([-1600 1600]);
xlabel(xaxis_text)
ylabel(yaxis_text)
title(title_text)
end
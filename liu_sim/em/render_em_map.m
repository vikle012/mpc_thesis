function render_em_map(em, line_grid, line_text_size, axis_text_size, text_spacing, xaxis_text, yaxis_text, title_text)
X = em.wix;
Y = em.Tix; % Replace with em.Tix.*em.wix for power plot
Z = em.eta;

% for i = 1:length(em.wix)
%     for j = 1:length(em.Tix)
%        if (em.Tix(j) >= em.Tmax(i)) || (em.Tix(j) <= em.Tmin(i))
%           Z(j,i) = 0; 
%        end
%     end
% end



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


fill([em.wix(1,1) em.wix(end,:) em.wix(end,end)], [1600; em.Tmax; 1600], 'w');
plot(X(1,:), em.Tmax', '-', 'Color', [1 0.5 0], 'LineWidth', 2) % .*em.wix for power plot
fill([em.wix(1,1) em.wix(end,:) em.wix(end,end)], [-1600; em.Tmin; -1600], 'w');
plot(X(1,:), em.Tmin', '-', 'Color', [1 0.5 0], 'LineWidth', 2) % .*em.wix for power plot

%ylim([-1600 1600]);
xlabel(xaxis_text)
ylabel(yaxis_text)
title(title_text)


end
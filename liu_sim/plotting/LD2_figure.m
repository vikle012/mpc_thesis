function [prev_fig_no, fig] = LD2_figure()
global LD2_fig

try
    prev_fig_no = LD2_fig;
    fig = figure(LD2_fig); LD2_incr;
catch
    % LD2_fig not initialized
    LD2_fig_init(1);
    prev_fig_no = 1;
    fig = figure(LD2_fig); LD2_incr;
end

end
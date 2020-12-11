function plot_engine_signal(sig_name, signals, line_width)

str_base = ['signals.', sig_name];

plot(signals.t_vec.val, eval([str_base, '.val']), 'LineWidth', line_width);
title ( eval([str_base, '.title']) );
ylabel( eval([str_base, '.label']) );
xlabel( signals.t_vec.label );

end
function litre_s = litrePerSecond(opt_signals)

m_fuel = trapz(opt_signals.t_vec.val, opt_signals.W_f.val);
litre_fuel = m_fuel / 0.745; % 0.745 kg/l diesel density
litre_s = litre_fuel / ...
    (opt_signals.t_vec.val(end) - opt_signals.t_vec.val(1));

end











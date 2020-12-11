function T = mix_wg_t(W_turb, W_wg, T_at, T_wg)
    T = (W_turb .* T_at + W_wg .* T_wg) ./ (W_turb + W_wg);
end


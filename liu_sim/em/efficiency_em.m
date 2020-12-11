%% KAN INTE ANVÄNDAS! ANV. LUT
function [eta_em] = efficiency_em(T_em, w_em, em)   % T = [-1500, 1500] w = 0-314
    [~, T_index] = min(abs( em.Tix - T_em ));
    [~, w_index] = min(abs( em.wix - w_em ));
    eta_em = em.eta(T_index, w_index);
end


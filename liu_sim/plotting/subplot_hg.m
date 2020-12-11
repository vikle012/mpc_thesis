function ax = subplot_hg(a, b, c)

if nargin == 1
    ax = subplot(a); hold on; grid on;
else
    ax = subplot(a, b, c); hold on; grid on;
end

end
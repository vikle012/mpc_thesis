function lut = load_lut(s0, sf, road_type, s_road, a_road)

if strcmp(road_type, 'flat')
    s_vec = linspace(s0, sf, 4);
    a_vec = zeros(1,4)+rand(1,4)*eps^10;
else
    s_vec = s_road;
    a_vec = a_road;
end

% Look-up table
lut = casadi.interpolant('LUT', 'bspline', {s_vec}, a_vec);

end
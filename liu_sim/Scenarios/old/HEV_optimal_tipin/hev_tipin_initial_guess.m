function v0 = hev_tipin_initial_guess(x0mm, u0, tf, param)
fprintf('Simulating initial guess...\n\n');

[u_f, u_thr, u_wg, M_em] = enum(4);

% Add step to control signal
u = [u0(u_f,1); ...    % ld2
    u0(u_thr,1); ...   % ld2
    u0(u_wg,1); ...    % ld2
    1e-6 ...     % M_em
    ];

x0 = [x0mm(1,1); ...   % ld2
    x0mm(2,1); ...     % ld2
    x0mm(3,1); ...     % ld2
    x0mm(4,1); ...     % ld2
    1e-6; ...          % Nox
    500; ...          % N
    0.375 ...           % SOC
    ];

% Simulate step
%tf = 6;
sim_model = @(t,x) hybridPowertrain_init_guess(x, u, t, param);
opts = odeset('RelTol', 1e-6, 'AbsTol', 1e-7);
[t_sim, x_sim] = ode15s(sim_model, [0, tf], x0, opts);

% Simulated control signal
u_sim = repmat(u', length(t_sim), 1);
lut_M = param.lut_M_WHTC;
u_guess = repmat([0 0 0 0], length(t_sim), 1); % Derivative of control


% pwl formulation
u_sim(1,1) = max(0, full(lut_M(t_sim(1))) ./ 10+8); % Removing bias results in optimum solution at 8 seconds. Only finds acceptable level with bias.
for i = 2:length(t_sim)
   u_sim(i,1) = max(0, full(lut_M(t_sim(i))) ./ 10+8);
   u_guess(i,1) = (u_sim(i,1) - u_sim(i-1,1)) ./ (t_sim(i) - t_sim(i-1));
end


x_guess = [x_sim, u_sim];

%plot(t_sim, x_guess(:,11)); hold on;
%plot(t_sim, u_guess(:,1))

v0 = yopInitialGuess(tf, t_sim, x_guess, u_guess, 0);
end


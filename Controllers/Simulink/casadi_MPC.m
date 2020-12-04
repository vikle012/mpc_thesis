classdef casadi_MPC < matlab.System & matlab.system.mixin.Propagates
    % untitled Add summary here
    %
    % This template includes the minimum set of functions required
    % to define a System object with discrete state.

    properties
        % Public, tunable properties.

    end

    properties (DiscreteState)
    end

    properties (Access = private)
        % Pre-computed constants.
        casadi_solver
        x0
        lbx
        ubx
        lbg
        ubg
    end

    methods (Access = protected)
        function num = getNumInputsImpl(~)
            num = 2;
        end
        function num = getNumOutputsImpl(~)
            num = 1;
        end
        function dt1 = getOutputDataTypeImpl(~)
        	dt1 = 'double';
        end
        function dt1 = getInputDataTypeImpl(~)
        	dt1 = 'double';
        end
        function sz1 = getOutputSizeImpl(~)
        	sz1 = [3,1];
        end
        function sz1 = getInputSizeImpl(~)
        	sz1 = [5,1];
        end
        function cp1 = isInputComplexImpl(~)
        	cp1 = false;
        end
        function cp1 = isOutputComplexImpl(~)
        	cp1 = false;
        end
        function fz1 = isInputFixedSizeImpl(~)
        	fz1 = true;
        end
        function fz1 = isOutputFixedSizeImpl(~)
        	fz1 = true;
        end
        function setupImpl(obj,~,~)
            load parameterData
            import casadi.*

            N = 25;    % Prediction horizon/number of control intervals
            Ts = 0.2;   % Sample time

            % From engine_map.m with m = 800
            x_opt = [156530.169403718; 162544.519591941; 0.228497904007297; ...
                     0.120767676163110; 6598.24892268606];
            u_opt = [110.976447879888; 15.3657549771663; 70.8311084176616];    

            % Declare external input signals
            n_e = 1500;     % Note: 500 <= n_e <= 2000

            x = casadi.SX.sym('X',5); % 5x1 matrix
            % p_im    = x(1);
            % p_em    = x(2);
            % X_Oim   = x(3);
            % X_Oem   = x(4);
            % w_t     = x(5);

            % Declare control signals
            u = casadi.SX.sym('U',3); % 3x1 matrix
            % u_delta = u(1);
            % u_egr   = u(2);
            % u_vgt   = u(3);

            % Linearizing system around x_opt and u_opt as stationary points
            % follows the format in "Reglerteori"
            A = casadi.Function('A', {x,u}, {jacobian(diesel_engine(x, u, n_e, model), x)});
            B = casadi.Function('B', {x,u}, {jacobian(diesel_engine(x, u, n_e, model), u)});
            xtilde = casadi.SX.sym('xtilde', 5);
            utilde = casadi.SX.sym('xtilde', 3);

            % Linearized continous time system
            xtilde_dot = A(x_opt, u_opt)*xtilde + B(x_opt, u_opt)*utilde;

            q1 = 10/((0.5*model.p_amb + 10*model.p_amb)/2)^2;
            q2 = 10/((0.5*model.p_amb + 20*model.p_amb)/2)^2;
            q3 = 10/((0 + 1)/2)^2;
            q4 = 10/((0 + 1)/2)^2;
            q5 = 10/((100*pi/30 + 200000*pi/30)/2)^2;

            r1 = 1/((1 + 250)/2)^2;
            r2 = 0.1/((0 + 100)/2)^2;
            r3 = 0.1/((20 + 100)/2)^2;

            Q1 = diag([q1 q2 q3 q4 q5]);
            Q2 = diag([r1 r2 r3]);

            % Objective function
            L = xtilde'*Q1*xtilde + utilde'*Q2*utilde;

            % Continuous time dynamics using diesel_engine.m
            f = casadi.Function('f', {xtilde, utilde}, {xtilde_dot, L});

            % Start with an empty NLP
            args.w   = [];
            args.w0  = [];
            args.lbw = [];
            args.ubw = [];
            args.J   = 0;
            args.G   = [];
            args.lbg = [];
            args.ubg = [];

            x_lbw = [0.5*model.p_amb; 0.5*model.p_amb;  0; 0; 100*pi/30]; 
            x_ubw = [10*model.p_amb;  20*model.p_amb;   1; 1; 200000*pi/30];
            u_lbw = [1; 0; 20];
            u_ubw = [250; 100; 100];
            xtilde_lbw = x_lbw - x_opt; 
            xtilde_ubw = x_ubw - x_opt;
            utilde_lbw = u_lbw - u_opt;
            utilde_ubw = u_ubw - u_opt;

            xtilde_dagger = [-150000; 0; 0; 0; 0]; % Disturbed state

            % "Lift" initial conditions
            X0 = casadi.MX.sym('X0', 5);
            args.w = [args.w; X0];
            args.lbw = [args.lbw; xtilde_dagger];
            args.ubw = [args.ubw; xtilde_dagger];
            args.w0 = [args.w0; xtilde_dagger];

            F = integration_function(f, "EF", Ts, N); % TODO: b?r det vara N h?r?

            % Formulate the NLP
            Xk = X0;
            for k=0:N-1        
                % New NLP variable for the control     
                Uk = casadi.MX.sym(['U_' num2str(k)], 3);
                args.w = [args.w; Uk];
                args.lbw = [args.lbw; utilde_lbw];
                args.ubw = [args.ubw; utilde_ubw]; 
                args.w0 = [args.w0; zeros(3,1)];

                % Integrate till the end of the interval
                Fk = F('x0', Xk, 'p', Uk);
                Xk_next = Fk.xf;
                args.J = args.J + Fk.qf;

                % New NLP variable for state at end of interval
                Xk = casadi.MX.sym(['X_' num2str(k+1)], 5);
                args.w = [args.w; Xk];
                args.lbw = [args.lbw; xtilde_lbw];
                args.ubw = [args.ubw; xtilde_ubw];
                args.w0 = [args.w0; zeros(5,1)];

                % Add equality constraint
                args.G = [args.G; Xk_next-Xk];
                args.lbg = [args.lbg; zeros(5,1)];
                args.ubg = [args.ubg; zeros(5,1)];  

            end

            % ipopt options
            opts = struct('ipopt',struct('print_level',0),'print_time',false);

            % Create an NLP solver function
            nlp = struct('f', args.J, 'x', args.w, 'g', args.G);
            solver = casadi.nlpsol('solver', 'ipopt', nlp, opts);

            obj.casadi_solver = solver;
            obj.x0 = args.w0;
            obj.lbx = args.lbw;
            obj.ubx = args.ubw;
            obj.lbg = args.lbg;
            obj.ubg = args.ubg;
        end

        function u = stepImpl(obj,x,t)
            %disp(t)
            
            x_opt = [156530.169403718; 162544.519591941; 0.228497904007297; ...
                     0.120767676163110; 6598.24892268606];
            u_opt = [110.976447879888; 15.3657549771663; 70.8311084176616];
            
            w0 = obj.x0;
            lbw = obj.lbx;
            ubw = obj.ubx;
            solver = obj.casadi_solver;
            lbw(1:5) = x(1:5) - x_opt;
            ubw(1:5) = x(1:5) - x_opt;
            sol = solver('x0', w0, 'lbx', lbw, 'ubx', ubw,...
                        'lbg', obj.lbg, 'ubg', obj.ubg);
            w_opt = full(sol.x);
            
            u = [w_opt(6) + u_opt(1);...
                 w_opt(7) + u_opt(2);...
                 w_opt(8) + u_opt(3)];
            %toc
        end

        function resetImpl(obj)
            % Initialize discrete-state properties.
        end
    end
end

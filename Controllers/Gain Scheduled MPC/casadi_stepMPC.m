classdef casadi_stepMPC < matlab.System & matlab.system.mixin.Propagates
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
            
            T = 5;
            N = 25;    % Prediction horizon/number of control intervals
            Ts = T/N;   % Sample time

            % From engine_map.m with m = 800
            x_opt = [163715.168088435; 167962.252867181; 0.229672106525997; ...
                     0.116324617241729; 6897.33534814885];
            u_opt = [122.033437081754; 12.7268611588264; 74.8549539702291];
            
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
            utilde = casadi.SX.sym('utilde', 3);
            utilde_old = casadi.SX.sym('utilde_old', 3);

            % Linearized continous time system
            xtilde_dot = A(x_opt, u_opt)*xtilde + B(x_opt, u_opt)*utilde;

            q1 = 20/((0.5*model.p_amb + 10*model.p_amb)/2)^2;
            q2 = 20/((0.5*model.p_amb + 20*model.p_amb)/2)^2;
            q3 = 10/((0 + 1)/2)^2;
            q4 = 10/((0 + 1)/2)^2;
            q5 = 20/((100*pi/30 + 200000*pi/30)/2)^2;

            r1 = 0.01/((1 + 250)/2)^2;
            r2 = 0.01/((0 + 100)/2)^2;
            r3 = 0.01/((20 + 100)/2)^2;

            Q1 = diag([q1 q2 q3 q4 q5]);
            Q2 = diag([r1 r2 r3]);

            % Objective function
            L = xtilde'*Q1*xtilde + ...
                (utilde - utilde_old)'*Q2*(utilde - utilde_old);

            % Continuous time dynamics using diesel_engine.m
            f = casadi.Function('f', {xtilde, utilde, utilde_old}, ...
                {xtilde_dot, L});

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

            xtilde_dagger = [0; 0; 0; 0; 0]; % Disturbed state

            % "Lift" initial conditions
            Uold = casadi.MX.sym('Uold', 3);
            args.w = [args.w; Uold];
            args.lbw = [args.lbw; [111.0829; 16.2702; 71.5151] - u_opt];
            args.ubw = [args.ubw; [111.0829; 16.2702; 71.5151] - u_opt];
            args.w0 = [args.w0; [111.0829; 16.2702; 71.5151] - u_opt];
            
            X0 = casadi.MX.sym('X0', 5);
            args.w = [args.w; X0];
            args.lbw = [args.lbw; xtilde_dagger];
            args.ubw = [args.ubw; xtilde_dagger];
            args.w0 = [args.w0; xtilde_dagger];

            M = 4; % Number of integration steps per sample
            F = integration_function(f, "EF", Ts, M);          
            
            % Formulate the NLP
            Xk = X0;
            Utilde_old = Uold;
            for k=0:N-1
                % New NLP variable for the control     
                Uk = casadi.MX.sym(['U_' num2str(k)], 3);
                args.w = [args.w; Uk];
                args.lbw = [args.lbw; utilde_lbw];
                args.ubw = [args.ubw; utilde_ubw]; 
                args.w0 = [args.w0; zeros(3,1)];              

                % Integrate till the end of the interval
                Fk = F('x0', Xk, 'p', Uk, 'p_old', Utilde_old);
                Xk_next = Fk.xf;
                args.J = args.J + Fk.qf;
                Utilde_old = Uk; % For integral action 
                
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

            % qpOASES options
            % opts = struct('qpoases', struct("setPrintLevel", "PL_NONE"));

            % Create an NLP solver function
            qp = struct('f', args.J, 'x', args.w, 'g', args.G);
            solver = casadi.qpsol('solver', 'qpoases', qp);

            obj.casadi_solver = solver;
            obj.x0 = args.w0;
            obj.lbx = args.lbw;
            obj.ubx = args.ubw;
            obj.lbg = args.lbg;
            obj.ubg = args.ubg;

        end

        function u = stepImpl(obj,x,t)                       
            x_opt = [163715.168088435; 167962.252867181; 0.229672106525997; ...
                     0.116324617241729; 6897.33534814885];
            u_opt = [122.033437081754; 12.7268611588264; 74.8549539702291];
            
            w0 = obj.x0;
            lbw = obj.lbx;
            ubw = obj.ubx;
            solver = obj.casadi_solver;
            lbw(4:8) = x - x_opt;
            ubw(4:8) = x - x_opt;
            sol = solver('x0', w0, 'lbx', lbw, 'ubx', ubw,...
                        'lbg', obj.lbg, 'ubg', obj.ubg);
            w_opt = full(sol.x);
            
            u = [w_opt(9) + u_opt(1);...
                 w_opt(10) + u_opt(2);...
                 w_opt(11) + u_opt(3)];
             
            % For integral action 
            obj.lbx(1:3) = u - u_opt;
            obj.ubx(1:3) = u - u_opt;
        end

        function resetImpl(obj)
            % Initialize discrete-state properties.
        end
    end
end

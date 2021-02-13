classdef L < matlab.System & matlab.system.mixin.Propagates
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
        u_old
    end

    methods (Access = protected)
        function num = getNumInputsImpl(~)
            num = 5;
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
        function setupImpl(obj,~,~,~,~,~)
            import casadi.*
            global init_param;
            
            model = init_param.model;
            T_s = init_param.T_s;
            N = init_param.N;
            
            % Weight matrices
            Q = init_param.Q;
            R = init_param.R;
            S = init_param.S;
            
            % 80 kW, 1200 rpm -> 636 Nm
            P = 80000;
            n_e = 1200;
            [x_0, u_0] = reference_generator(P/(n_e*pi/30), n_e);
            
            x_lbw = [0.5*model.p_amb; 0.5*model.p_amb;  0; 0; 100*pi/30]; 
            x_ubw = [10*model.p_amb;  20*model.p_amb;   1; 1; 200000*pi/30];
            u_lbw = [1; 0; 20];
            u_ubw = [250; 100; 100];
            
            % SX variables
            X = casadi.SX.sym('X', 5);
            U = casadi.SX.sym('U', 3);
            X_ref = casadi.SX.sym('X_ref', 5);
            U_ref = casadi.SX.sym('U_ref', 3);
            U_old = casadi.SX.sym('U_old', 3);
            
            % Functions
            A = casadi.Function('A', {X, U}, {jacobian(diesel_engine(X, U, n_e, model), X)});
            B = casadi.Function('B', {X, U}, {jacobian(diesel_engine(X, U, n_e, model), U)});
            K_c = casadi.Function('K_c', {X, U}, {diesel_engine(X, U, n_e, model)});

            % Linearized continous time system
            Xtilde_dot = A(x_0, u_0)*(X - x_0) + B(x_0, u_0)*(U - u_0) + K_c(x_0, u_0);

            % Objective function
            L = (X - X_ref)'*Q*(X - X_ref) + ... \
                (U - U_ref)'*R*(U - U_ref) + ...
                init_param.integral_action*(U - U_old)'*S*(U - U_old);

            % Continuous time dynamics using diesel_engine.m
            f = casadi.Function('f', ...
                {X, U, X_ref, U_ref, U_old}, {Xtilde_dot, L});

            % Start with an empty NLP
            args.w   = [];
            args.w0  = [];
            args.lbw = [];
            args.ubw = [];
            args.J   = 0;
            args.G   = [];
            args.lbg = [];
            args.ubg = [];
            args.p   = [];

            X_ref = casadi.MX.sym('X_ref', 5);
            args.p = [args.p; X_ref];
            
            U_ref = casadi.MX.sym('U_ref', 3);
            args.p = [args.p; U_ref];
            
            % "Lift" initial conditions
            U_old = casadi.MX.sym('U_old', 3);
            args.p = [args.p; U_old];
            obj.u_old = init_param.u_old;
            
            X_0 = casadi.MX.sym('X_0', 5);
            args.w = [args.w; X_0];
            args.lbw = [args.lbw; zeros(5,1)];
            args.ubw = [args.ubw; zeros(5,1)];
            args.w0 = [args.w0; zeros(5,1)];        
            
            % Formulate the NLP
            X_k = X_0;
            for k=0:N-1
                % New NLP variable for the control     
                U_k = casadi.MX.sym(['U_' num2str(k)], 3);
                args.w = [args.w; U_k];
                args.lbw = [args.lbw; u_lbw];
                args.ubw = [args.ubw; u_ubw]; 
                args.w0 = [args.w0; init_param.u_old];  

                % Integrate using Euler Forward
                [dx, q_k] = f(X_k, U_k, X_ref, U_ref, U_old);
                X_k_next = X_k + T_s*dx;
                args.J = args.J + T_s*q_k;
                              
                U_old = U_k; % For integral action
                
                % New NLP variable for state at end of interval
                X_k = casadi.MX.sym(['X_' num2str(k+1)], 5);
                args.w = [args.w; X_k];
                args.lbw = [args.lbw; x_lbw];
                args.ubw = [args.ubw; x_ubw];
                args.w0 = [args.w0; zeros(5,1)];       

                % Add equality constraint
                args.G = [args.G; X_k_next - X_k];
                args.lbg = [args.lbg; zeros(5,1)];
                args.ubg = [args.ubg; zeros(5,1)];    
            end
            
            % Last state objective term
            V_f = (X_k - X_ref)'*Q*(X_k - X_ref);
            args.J = args.J + T_s*V_f;
            
            % Create an NLP solver function
            qp = struct('f', args.J, 'x', args.w, 'p', args.p, 'g', args.G);
            solver = casadi.qpsol('solver', 'qpoases', qp);

            obj.casadi_solver = solver;
            obj.x0 = args.w0;
            obj.lbx = args.lbw;
            obj.ubx = args.ubw;
            obj.lbg = args.lbg;
            obj.ubg = args.ubg;
        end

        function u = stepImpl(obj,x,t,n_e,x_ref,u_ref)
            tic
            
            if x_ref(1) < 0
                obj.u_old = [1; obj.u_old(2:3)];
                u = obj.u_old;
            else
                w0 = obj.x0;
                lbw = obj.lbx;
                ubw = obj.ubx;
                solver = obj.casadi_solver;

                p = [x_ref; u_ref];
                p = [p; obj.u_old];

                lbw(1:5) = x;
                ubw(1:5) = x;

                sol = solver('x0', w0, ...
                             'p', p, ...
                             'lbx', lbw, ...
                             'ubx', ubw,...
                             'lbg', obj.lbg, ...
                             'ubg', obj.ubg);
                w_opt = full(sol.x);
                u_opt = w_opt(6:8);

                u = u_opt;

                % For integral action
                obj.u_old = u;
            end
            toc
        end

        function resetImpl(obj)
            % Initialize discrete-state properties.
        end
    end
end

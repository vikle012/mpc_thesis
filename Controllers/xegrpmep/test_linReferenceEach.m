classdef test_linReferenceEach < matlab.System & matlab.system.mixin.Propagates
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
        N
        constr
        func
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
            Q = init_param.Q;
            R = init_param.R;
            obj.N = init_param.N;
            
            x_lbw = [0.5*model.p_amb; 0.5*model.p_amb; 0; 0; 100*pi/30]; 
            x_ubw = [10*model.p_amb; 20*model.p_amb; 1; 1; 200000*pi/30];
            u_lbw = [1; 0; 20];
            u_ubw = [250; 100; 100];
            
            A = casadi.SX.sym('A', 5, 5);
            B = casadi.SX.sym('B', 5, 3);
            X = casadi.SX.sym('X', 5);
            U = casadi.SX.sym('U', 3);
            X_ref = casadi.SX.sym('X_ref', 5);
            U_ref = casadi.SX.sym('U_ref', 3);
            U_old = casadi.SX.sym('U_old', 3);
            
            % For x_egr linearization
            A_x_egr = casadi.SX.sym('A_x_egr', 1, 5);
            B_x_egr = casadi.SX.sym('B_x_egr', 1, 3);
            K_c_x_egr = casadi.SX.sym('K_c_x_egr');
            x_egr_ref = casadi.SX.sym('x_egr_ref');

            x_egr_lin = A_x_egr*(X - X_ref) + B_x_egr*(U - U_ref) + K_c_x_egr;
            
            % For lambda_O linearization
            A_lambda_O = casadi.SX.sym('A_lambda_O', 1, 5);
            B_lambda_O = casadi.SX.sym('B_lambda_O', 1, 3);
            K_c_lambda_O = casadi.SX.sym('B_lambda_O');
            
            lambda_O_lin = casadi.Function('lambda_O', {A_lambda_O, B_lambda_O, K_c_lambda_O, X, U, X_ref, U_ref}, ...
                {A_lambda_O*(X - X_ref) + B_lambda_O*(U - U_ref)+ K_c_lambda_O});
            
            % Linearized continous time system
            Xtilde_dot = A*(X - X_ref) + B*(U - U_ref);

            % Objective function
            L = (x_egr_lin - x_egr_ref)'*10*(x_egr_lin - x_egr_ref)+ ...
                0.001*(X(2) - X(1)) + ...
                (U - U_old)'*R*(U - U_old);
            
            f = casadi.Function('f', ...
                {A, B, X, U, X_ref, U_ref, U_old, A_x_egr, B_x_egr, K_c_x_egr, x_egr_ref}, {Xtilde_dot, L});
             
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
            
            A = casadi.MX.sym('A', 5, 5);
            args.p = [args.p; A(:)];
            
            B = casadi.MX.sym('B', 5, 3);
            args.p = [args.p; B(:)];
            
            X_ref = casadi.MX.sym('X_ref', 5);
            args.p = [args.p; X_ref];
            
            U_ref = casadi.MX.sym('U_ref', 3);
            args.p = [args.p; U_ref];
            
            U_old = casadi.MX.sym('Utilde_old', 3);
            args.p = [args.p; U_old];
            obj.u_old = init_param.u_old;
            
            % x_egr parameters
            A_x_egr = casadi.MX.sym('A_x_egr', 1, 5);
            args.p = [args.p; A_x_egr(:)];
            B_x_egr = casadi.MX.sym('B_x_egr', 1, 3);
            args.p = [args.p; B_x_egr(:)];
            K_c_x_egr = casadi.MX.sym('K_c_x_egr');
            args.p = [args.p; K_c_x_egr];
            x_egr_ref = casadi.MX.sym('x_egr_ref');
            args.p = [args.p; x_egr_ref];
            
            % lambda_O parameters
            A_lambda_O = casadi.MX.sym('A_lambda_O', 1, 5);
            args.p = [args.p; A_lambda_O(:)];
            B_lambda_O = casadi.MX.sym('B_lambda_O', 1, 3);
            args.p = [args.p; B_lambda_O(:)]; 
            K_c_lambda_O = casadi.MX.sym('K_c_lambda_O');
            args.p = [args.p; K_c_lambda_O];            
            lambda_O_ref = casadi.MX.sym('lambda_O_ref');
            args.p = [args.p; lambda_O_ref];
            
            % Initialize OCP            
            X_0 = casadi.MX.sym('X_0', 5);
            args.w = [args.w; X_0];
            args.lbw = [args.lbw; zeros(5,1)];
            args.ubw = [args.ubw; zeros(5,1)];
            args.w0 = [args.w0; zeros(5,1)];
            
            % Formulate the NLP
            X_k = X_0;
            for k=0:obj.N-1
                % New NLP variable for the control     
                U_k = casadi.MX.sym(['Utilde_' num2str(k)], 3);
                args.w = [args.w; U_k];
                args.lbw = [args.lbw; u_lbw];
                args.ubw = [args.ubw; u_ubw]; 
                args.w0 = [args.w0; zeros(3,1)];  

                % Integrate using Euler Forward
                [dx, q_k] = f(A, B, X_k, U_k, X_ref, U_ref, init_param.integral_action*U_old, A_x_egr, B_x_egr, K_c_x_egr, x_egr_ref);
                X_k_next = X_k + T_s*dx;
                args.J = args.J + T_s*q_k;
                              
                U_old = U_k; % For integral action
                
                lambda_O = lambda_O_lin(A_lambda_O, B_lambda_O, K_c_lambda_O, X_k, U_k, X_ref, U_ref);
                args.G = [args.G; lambda_O_ref - lambda_O];
                args.lbg = [args.lbg; 0];
                args.ubg = [args.ubg; inf]; 
                
                % New NLP variable for state at end of interval
                X_k = casadi.MX.sym(['Xtilde_' num2str(k+1)], 5);
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
            % V_f = (X_k - X_ref)'*Q*(X_k - X_ref);
            % args.J = args.J + T_s*V_f;
            
            % Create an NLP solver function
            qp = struct('f', args.J, 'x', args.w, 'p', args.p, 'g', args.G);
            solver = casadi.qpsol('solver', 'qpoases', qp);
            
            obj.casadi_solver = solver;
            obj.x0 = args.w0;
            obj.lbx = args.lbw;
            obj.ubx = args.ubw;
            obj.lbg = args.lbg;
            obj.ubg = args.ubg; 
            
            X = casadi.SX.sym('X', 5);
            U = casadi.SX.sym('U', 3);
            n_e = casadi.SX.sym('n_e');
            obj.func.A = casadi.Function('A', {X, U, n_e}, ...
                {jacobian(diesel_engine(X, U, n_e, model), X)});
            obj.func.B = casadi.Function('B', {X, U, n_e}, ...
                {jacobian(diesel_engine(X, U, n_e, model), U)});
            
            [~, y, ~] = diesel_engine(X, U, n_e, model);
            obj.func.A_x_egr = casadi.Function('A_x_egr', {X, U, n_e}, ...
                {jacobian(y.x_egr, X)});
            obj.func.B_x_egr = casadi.Function('B_x_egr', {X, U, n_e}, ...
                {jacobian(y.x_egr, U)});
            obj.func.K_c_x_egr = casadi.Function('K_c_x_egr', {X, U, n_e}, ...
                {y.x_egr});
            
            obj.func.A_lambda_O = casadi.Function('A_x_egr', {X, U, n_e}, ...
                {jacobian(y.lambda_O, X)});
            obj.func.B_lambda_O = casadi.Function('B_x_egr', {X, U, n_e}, ...
                {jacobian(y.lambda_O, U)});
            obj.func.K_c_lambda_O = casadi.Function('K_c_x_egr', {X, U, n_e}, ...
                {y.lambda_O});
        end

        function u = stepImpl(obj,x,t,n_e,x_ref,u_ref)  
            tic

            w0 = obj.x0;
            lbw = obj.lbx;
            ubw = obj.ubx;            
            solver = obj.casadi_solver;
            
            % Linearize
            A = obj.func.A(x_ref, u_ref, n_e);
            B = obj.func.B(x_ref, u_ref, n_e);
            % Independent parameters
            p = full(A(:));                 % A = 5x5
            p = [p; full(B(:))];            % B = 5x3
            p = [p; x_ref];
            p = [p; u_ref];
            % p = [p; n_e];
            p = [p; obj.u_old];     % Utilde_old = 3x1
            
            % Linearize x_egr
            A_x_egr = obj.func.A_x_egr(x_ref, u_ref, n_e);
            B_x_egr = obj.func.B_x_egr(x_ref, u_ref, n_e);
            K_c_x_egr = obj.func.K_c_x_egr(x_ref, u_ref, n_e);
            x_egr_ref = obj.func.K_c_x_egr(x_ref, u_ref, n_e);
            p = [p; full(A_x_egr(:)); full(B_x_egr(:)); K_c_x_egr; x_egr_ref];

            % Linearize x_egr
            A_lambda_O = obj.func.A_lambda_O(x_ref, u_ref, n_e);
            B_lambda_O = obj.func.B_lambda_O(x_ref, u_ref, n_e);
            K_c_lambda_O = obj.func.K_c_lambda_O(x_ref, u_ref, n_e);
            lambda_O_ref = obj.func.K_c_lambda_O(x_ref, u_ref, n_e);
            p = [p; full(A_lambda_O(:)); full(B_lambda_O(:)); K_c_lambda_O; lambda_O_ref];
            
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
            toc
        end

        function resetImpl(obj)
            % Initialize discrete-state properties.
        end
    end
end

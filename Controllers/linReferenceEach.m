classdef linReferenceEach < matlab.System & matlab.system.mixin.Propagates
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
            
            obj.constr.x_lbw = [0.5*model.p_amb; 0.5*model.p_amb; 0; 0; 100*pi/30]; 
            obj.constr.x_ubw = [10*model.p_amb; 20*model.p_amb; 1; 1; 200000*pi/30];
            obj.constr.u_lbw = [1; 0; 20];
            obj.constr.u_ubw = [250; 100; 100];
            
            A = casadi.SX.sym('A', 5, 5);
            B = casadi.SX.sym('B', 5, 3);
            Xtilde = casadi.SX.sym('Xtilde', 5);
            Utilde = casadi.SX.sym('Utilde', 3);
            Utilde_old = casadi.SX.sym('Utilde_old', 3);
            
            % Linearized continous time system
            Xtilde_dot = A*Xtilde + B*Utilde;

            % Objective function
            L = Xtilde'*Q*Xtilde + ...
                (Utilde - Utilde_old)'*R*(Utilde - Utilde_old);
            
            f = casadi.Function('f', ...
                {A, B, Xtilde, Utilde, Utilde_old}, {Xtilde_dot, L});
             
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
            
            % These constraints will be overwritten each iteration
            xtilde_lbw = obj.constr.x_lbw;
            xtilde_ubw = obj.constr.x_ubw;
            utilde_lbw = obj.constr.u_lbw;
            utilde_ubw = obj.constr.u_ubw;
            
            A = casadi.MX.sym('A', 5, 5);
            args.p = [args.p; A(:)];
            
            B = casadi.MX.sym('B', 5, 3);
            args.p = [args.p; B(:)];
            
            Utilde_old = casadi.MX.sym('Utilde_old', 3);
            args.p = [args.p; Utilde_old];
            obj.u_old = init_param.u_old;
            
            % Initialize OCP            
            Xtilde_0 = casadi.MX.sym('Xtilde_0', 5);
            args.w = [args.w; Xtilde_0];
            args.lbw = [args.lbw; zeros(5,1)];
            args.ubw = [args.ubw; zeros(5,1)];
            args.w0 = [args.w0; zeros(5,1)];
            
            % Formulate the NLP
            Xtilde_k = Xtilde_0;
            for k=0:obj.N-1
                % New NLP variable for the control     
                Utilde_k = casadi.MX.sym(['Utilde_' num2str(k)], 3);
                args.w = [args.w; Utilde_k];
                args.lbw = [args.lbw; utilde_lbw];
                args.ubw = [args.ubw; utilde_ubw]; 
                args.w0 = [args.w0; zeros(3,1)];  

                % Integrate using Euler Forward
                [dx, q_k] = f(A, B, Xtilde_k, Utilde_k, Utilde_old);
                Xtilde_k_next = Xtilde_k + T_s*dx;
                args.J = args.J + T_s*q_k;
                              
                Utilde_old = Utilde_k; % For integral action
                
                % New NLP variable for state at end of interval
                Xtilde_k = casadi.MX.sym(['Xtilde_' num2str(k+1)], 5);
                args.w = [args.w; Xtilde_k];
                args.lbw = [args.lbw; xtilde_lbw];
                args.ubw = [args.ubw; xtilde_ubw];
                args.w0 = [args.w0; zeros(5,1)];       

                % Add equality constraint
                args.G = [args.G; Xtilde_k_next - Xtilde_k];
                args.lbg = [args.lbg; zeros(5,1)];
                args.ubg = [args.ubg; zeros(5,1)];  
            end
            
            % Last state objective term
            V_f = Xtilde_k'*Q*Xtilde_k;
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
            
            X = casadi.SX.sym('X', 5);
            U = casadi.SX.sym('U', 3);
            n_e = casadi.SX.sym('n_e');
            obj.func.A = casadi.Function('A', {X, U, n_e}, ...
                {jacobian(diesel_engine(X, U, n_e, model), X)});
            obj.func.B = casadi.Function('B', {X, U, n_e}, ...
                {jacobian(diesel_engine(X, U, n_e, model), U)});
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
            p = [p; obj.u_old - u_ref];     % Utilde_old = 3x1

            lbw(1:5) = x - x_ref;
            ubw(1:5) = x - x_ref;
            
            % Change all decision variable boundries
            for i = 0:obj.N-1
                lbw(6+8*i:8+8*i) = obj.constr.u_lbw - u_ref;
                ubw(6+8*i:8+8*i) = obj.constr.u_ubw - u_ref;              
                lbw(9+8*i:13+8*i) = obj.constr.x_lbw - x_ref;
                ubw(9+8*i:13+8*i) = obj.constr.x_ubw - x_ref;
            end
            
            sol = solver('x0', w0, ...
                         'p', p, ...
                         'lbx', lbw, ...
                         'ubx', ubw,...
                         'lbg', obj.lbg, ...
                         'ubg', obj.ubg);
            wtilde_opt = full(sol.x);
            utilde_opt = wtilde_opt(6:8);
            
            u = utilde_opt + u_ref;
            
            % For integral action
            obj.u_old = u;
            toc
        end

        function resetImpl(obj)
            % Initialize discrete-state properties.
        end
    end
end

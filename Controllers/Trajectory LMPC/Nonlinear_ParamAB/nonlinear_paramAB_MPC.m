classdef nonlinear_paramAB_MPC < matlab.System & matlab.system.mixin.Propagates
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
            
            X = casadi.SX.sym('X', 5);
            U = casadi.SX.sym('U', 3);
            U_old = casadi.SX.sym('U_old', 3);
            n_e = casadi.SX.sym('n_e');
            
            % Nonlinear continous time system dynamics
            [X_dot, ~, ~] = diesel_engine(X, U, n_e, model);

            % Objective function
            L = X'*Q*X + (U - U_old)'*R*(U - U_old);
            
            f = casadi.Function('f', {X, U, U_old, n_e}, {X_dot, L});
             
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
            x_lbw = obj.constr.x_lbw;
            x_ubw = obj.constr.x_ubw;
            u_lbw = obj.constr.u_lbw;
            u_ubw = obj.constr.u_ubw;
            
            n_e = casadi.MX.sym('n_e');
            args.p = [args.p; n_e];
            
            U_old = casadi.MX.sym('Utilde_old', 3);
            args.p = [args.p; U_old];
            obj.u_old = init_param.u_old;
            
            % Initialize OCP            
            X_0 = casadi.MX.sym('Xtilde_0', 5);
            args.w = [args.w; X_0];
            args.lbw = [args.lbw; zeros(5,1)];
            args.ubw = [args.ubw; zeros(5,1)];
            args.w0 = [args.w0; zeros(5,1)];
            
            % Formulate the NLP
            X_k = X_0;
            for k=0:obj.N-1
                % New NLP variable for the control     
                U_k = casadi.MX.sym(['U_' num2str(k)], 3);
                args.w = [args.w; U_k];
                args.lbw = [args.lbw; u_lbw];
                args.ubw = [args.ubw; u_ubw]; 
                args.w0 = [args.w0; zeros(3,1)];  

                % Integrate using Euler Forward
                [dx, q_k] = f(X_k, U_k, U_old, n_e);
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
            V_f = X_k'*Q*X_k;
            args.J = args.J + T_s*V_f;
            
            % Create an NLP solver function
            nlp = struct('f', args.J, 'x', args.w, 'p', args.p, 'g', args.G);
            solver = casadi.nlpsol('solver', 'ipopt', nlp);
            
            obj.casadi_solver = solver;
            obj.x0 = args.w0;
            obj.lbx = args.lbw;
            obj.ubx = args.ubw;
            obj.lbg = args.lbg;
            obj.ubg = args.ubg; 
        end

        function u = stepImpl(obj,x,t,n_e,x_ref,u_ref)  
            tic

            w0 = obj.x0;
            lbw = obj.lbx;
            ubw = obj.ubx;            
            solver = obj.casadi_solver; 
            p = [n_e; obj.u_old];

            lbw(1:5) = x;
            ubw(1:5) = x;
            
            % Change all decision variable boundries
            for i = 0:obj.N-1
                w0(6+8*i:8+8*i) = u_ref;
                w0(9+8*i:13+8*i) = x_ref;
            end
            
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

classdef each_MPC < matlab.System & matlab.system.mixin.Propagates
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
        param
        constr
        utilde_old
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
            
            tic
            load parameterData
            import casadi.*
            global tuning_param;
            
            obj.param.model = model;
            obj.param.N = tuning_param.N;    % Prediction horizon/number of control intervals
            obj.param.T_s = tuning_param.T_s;   % Sample time
            obj.param.Q = tuning_param.Q;
            obj.param.R = tuning_param.R;
            
            obj.constr.x_lbw = [0.5*obj.param.model.p_amb; 0.5*obj.param.model.p_amb; 0; 0; 100*pi/30]; 
            obj.constr.x_ubw = [10*obj.param.model.p_amb; 20*obj.param.model.p_amb; 1; 1; 200000*pi/30];
            obj.constr.u_lbw = [1; 0; 20];
            obj.constr.u_ubw = [250; 100; 100];
            
            obj.utilde_old = zeros(3,1);
          
        end

        function u = stepImpl(obj,x,t,n_e,x_ref,u_ref)  
            tic
            X = casadi.SX.sym('X', 5);
            U = casadi.SX.sym('U', 3);
            Utilde_old = casadi.SX.sym('Utilde_old', 3);

            A = casadi.Function('A', {X, U}, {jacobian(diesel_engine(X, U, n_e, obj.param.model), X)});
            B = casadi.Function('B', {X, U}, {jacobian(diesel_engine(X, U, n_e, obj.param.model), U)});
            Xtilde = casadi.SX.sym('Xtilde', 5);
            Utilde = casadi.SX.sym('Utilde', 3);
            
            % Linearized continous time system
            Xtilde_dot = A(x_ref, u_ref)*Xtilde + B(x_ref, u_ref)*Utilde;

            % Objective function
            L = Xtilde'*obj.param.Q*Xtilde + ...
                (Utilde - Utilde_old)'*obj.param.R*(Utilde - Utilde_old);
            
            f = casadi.Function('f', {Xtilde, Utilde, Utilde_old}, {Xtilde_dot, L});
             
            % Start with an empty NLP
            args.w   = [];
            args.w0  = [];
            args.lbw = [];
            args.ubw = [];
            args.J   = 0;
            args.G   = [];
            args.lbg = [];
            args.ubg = [];
            
            xtilde_lbw = obj.constr.x_lbw - x_ref; 
            xtilde_ubw = obj.constr.x_ubw - x_ref;
            utilde_lbw = obj.constr.u_lbw - u_ref;
            utilde_ubw = obj.constr.u_ubw - u_ref;
            
            % Initialize OCP            
            Xtilde_0 = casadi.MX.sym('Xtilde_0', 5);
            args.w = [args.w; Xtilde_0];
            args.lbw = [args.lbw; x - x_ref];
            args.ubw = [args.ubw; x - x_ref];
            args.w0 = [args.w0; x - x_ref];
            
            % Formulate the NLP
            Xtilde_k = Xtilde_0;
            Utilde_old = obj.utilde_old; % updated each iteration
            for k=0:obj.param.N-1
                % New NLP variable for the control     
                Utilde_k = casadi.MX.sym(['Utilde_' num2str(k)], 3);
                args.w = [args.w; Utilde_k];
                args.lbw = [args.lbw; utilde_lbw];
                args.ubw = [args.ubw; utilde_ubw]; 
                args.w0 = [args.w0; zeros(3,1)];  

                % Integrate using Euler Forward
                [dx, q_k] = f(Xtilde_k, Utilde_k, Utilde_old);
                Xtilde_k_next = Xtilde_k + obj.param.T_s*dx;
                args.J = args.J + obj.param.T_s*q_k;
                              
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
            qp = struct('f', args.J, 'x', args.w, 'g', args.G);
            solver = casadi.qpsol('solver', 'qpoases', qp);
            sol = solver('x0', args.w0, 'lbx', args.lbw, 'ubx', args.ubw,...
                         'lbg', args.lbg, 'ubg', args.ubg);
            wtilde_opt = full(sol.x);
            utilde_opt = wtilde_opt(6:8);
            % For integral action
            obj.utilde_old = utilde_opt;
            
            u = utilde_opt + u_ref;
            toc
        end

        function resetImpl(obj)
            % Initialize discrete-state properties.
        end
    end
end

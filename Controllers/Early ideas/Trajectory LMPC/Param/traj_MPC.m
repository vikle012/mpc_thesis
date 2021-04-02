classdef traj_MPC < matlab.System & matlab.system.mixin.Propagates
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
        u_ref_old
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

            x_ref_init = [156530.169403718; 162544.519591941; 0.228497904007297; ...
                          0.120767676163110; 6598.24892268606];
            u_ref_init = [110.976447879888; 15.3657549771663; 70.8311084176616];
            
            N = tuning_param.N;    % Prediction horizon/number of control intervals
            T_s = tuning_param.T_s;   % Sample time
            n_e_init = tuning_param.n_e;  

            X = casadi.SX.sym('X', 5); % 5x1 matrix
            X_ref = casadi.SX.sym('X_ref', 5);

            U = casadi.SX.sym('U', 3); % 3x1 matrix
            U_ref = casadi.SX.sym('U_ref', 3);

            n_e = casadi.SX.sym('n_e');
            
            U_old = casadi.SX.sym('U_old', 3);
            U_ref_old = casadi.SX.sym('U_ref_old', 3);

            % Linearized continous time system
            A = casadi.Function('A', {X, U, n_e}, {jacobian(diesel_engine(X, U, n_e, model), X)});
            B = casadi.Function('B', {X, U, n_e}, {jacobian(diesel_engine(X, U, n_e, model), U)});
            
            xtilde_dot = A(X_ref, U_ref, n_e)*(X - X_ref) + ...
                         B(X_ref, U_ref, n_e)*(U - U_ref);

            Q1 = tuning_param.Q;
            Q2 = tuning_param.R;

            % Objective function
            L = (X - X_ref)'*Q1*(X - X_ref) + ...
                (U - U_ref - (U_old - U_ref_old))'*Q2*(U - U_ref - (U_old - U_ref_old));

            f = casadi.Function('f', {X, X_ref, U, U_ref, U_old, U_ref_old, n_e}, ...
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
            args.p   = [];

            x_lbw = [0.5*model.p_amb; 0.5*model.p_amb;  0; 0; 100*pi/30]; 
            x_ubw = [10*model.p_amb;  20*model.p_amb;   1; 1; 200000*pi/30];
            u_lbw = [1; 0; 20];
            u_ubw = [250; 100; 100];
            
            % Initialize OCP
            U_old = casadi.MX.sym('U_old', 3);
            args.p = [args.p; U_old];
%             args.lbw = [args.lbw; u_ref_init];
%             args.ubw = [args.ubw; u_ref_init];
%             args.w0 = [args.w0; u_ref_init];
            U_ref_old = casadi.MX.sym('U_ref_old', 3);
            args.p = [args.p; U_ref_old];
            
            X_ref = casadi.MX.sym('X_ref', 5);
            args.p = [args.p; X_ref];
%             args.lbw = [args.lbw; x_ref_init];
%             args.ubw = [args.ubw; x_ref_init];
%             args.w0 = [args.w0; x_ref_init];
            
            U_ref = casadi.MX.sym('U_ref', 3);
            args.p = [args.p; U_ref];
%             args.lbw = [args.lbw; u_ref_init];
%             args.ubw = [args.ubw; u_ref_init];
%             args.w0 = [args.w0; u_ref_init];
            
            n_e = casadi.MX.sym('n_e');
            args.p = [args.p; n_e];
%             args.lbw = [args.lbw; n_e_init];
%             args.ubw = [args.ubw; n_e_init];
%             args.w0 = [args.w0; n_e_init];
            
            X_0 = casadi.MX.sym('X_0', 5);
            args.w = [args.w; X_0];
            args.lbw = [args.lbw; x_ref_init];
            args.ubw = [args.ubw; x_ref_init];
            args.w0 = [args.w0; x_ref_init];
            
            % Formulate the NLP
            X_k = X_0;
            % Utilde_old = U_old;
            for k=0:N-1
                % New NLP variable for the control     
                U_k = casadi.MX.sym(['U_' num2str(k)], 3);
                args.w = [args.w; U_k];
                args.lbw = [args.lbw; u_lbw];
                args.ubw = [args.ubw; u_ubw]; 
                args.w0 = [args.w0; u_ref_init];  

                % Integrate using Euler Forward
                [dx, q_k] = f(X_k, X_ref, U_k, U_ref, U_old, U_ref_old, n_e);
                Xk_next = X_k + T_s*dx;
                args.J = args.J + T_s*q_k;
                              
                U_old = U_k; % For integral action
                
                % New NLP variable for state at end of interval
                X_k = casadi.MX.sym(['X_' num2str(k+1)], 5);
                args.w = [args.w; X_k];
                args.lbw = [args.lbw; x_lbw];
                args.ubw = [args.ubw; x_ubw];
                args.w0 = [args.w0; x_ref_init];       

                % Add equality constraint
                args.G = [args.G; Xk_next-X_k];
                args.lbg = [args.lbg; zeros(5,1)];
                args.ubg = [args.ubg; zeros(5,1)];  
            end

            % qpOASES options
            % opts = struct('qpoases', struct("setPrintLevel", "PL_NONE"));
            
            % Create an NLP solver function
            qp = struct('f', args.J, 'x', args.w, 'p', args.p, 'g', args.G);
            solver = casadi.qpsol('solver', 'qpoases', qp);
            toc
            obj.casadi_solver = solver;
            obj.x0 = args.w0;
            obj.lbx = args.lbw;
            obj.ubx = args.ubw;
            obj.lbg = args.lbg;
            obj.ubg = args.ubg;
            
            obj.u_old = u_ref_init;
            obj.u_ref_old = u_ref_init;

        end

        function u = stepImpl(obj,x,t,n_e,x_ref,u_ref)  
            % 1-3 u_old
            % 4-8 x_ref
            % 9-11 u_ref
            % 12 n_e
            % 13-17
            % 18-20 u_0             
            w0 = obj.x0;
            lbw = obj.lbx;
            ubw = obj.ubx;
            solver = obj.casadi_solver;
            
            p = [obj.u_old; obj.u_ref_old; x_ref; u_ref; n_e];
            
%             % x_ref
%             lbw(4:8) = x_ref;
%             ubw(4:8) = x_ref;
%             % w0(4:8) = x_ref;
%             
%             % u_ref
%             lbw(9:11) = u_ref;
%             ubw(9:11) = u_ref;
%             % w0(9:11) = u_ref;
%             
%             % n_e
%             lbw(12) = n_e;
%             ubw(12) = n_e;
%             % w0(12) = n_e;
%             
            % x_0
            lbw(1:5) = x;
            ubw(1:5) = x;
            
            sol = solver('x0', w0, 'p', p, 'lbx', lbw, 'ubx', ubw,...
                        'lbg', obj.lbg, 'ubg', obj.ubg);
            w_opt = full(sol.x);
            
            u = w_opt(6:8);
            
            % For integral action Utilde_old = U_old - U_ref_old
            obj.u_old = u;
            obj.u_ref_old = u_ref;
            
        end

        function resetImpl(obj)
            % Initialize discrete-state properties.
        end
    end
end

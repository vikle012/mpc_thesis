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
        x_ref
        u_ref
        u_old
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
            addpath('C:\Users\Jonte\Documents\GitHub\mpc_thesis\WahlstromErikssonTCDI_EGR_VGT')
            load parameterData
            import casadi.*
            global ref;
            global tuning_param;
            
            obj.x_ref = ref.x_ref;
            obj.u_ref = ref.u_ref;
            
            N = tuning_param.N;    % Prediction horizon/number of control intervals
            T_s = tuning_param.T_s;   % Sample time
            n_e = tuning_param.n_e;  
                
            x_from = ref.x_start;
            u_from = ref.u_start;
            x_to = ref.x_ref;
            u_to = ref.u_ref;

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

            A = casadi.Function('A', {x,u}, {jacobian(diesel_engine(x, u, n_e, model), x)});
            B = casadi.Function('B', {x,u}, {jacobian(diesel_engine(x, u, n_e, model), u)});
            xtilde = casadi.SX.sym('xtilde', 5);
            utilde = casadi.SX.sym('utilde', 3);
            utilde_old = casadi.SX.sym('utilde_old', 3);

            % Linearized continous time system
            xtilde_dot = A(x_to, u_to)*xtilde + B(x_to, u_to)*utilde;

            Q1 = tuning_param.Q;
            Q2 = tuning_param.R;

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
            args.p   = [];

            x_lbw = [0.5*model.p_amb; 0.5*model.p_amb;  0; 0; 100*pi/30]; 
            x_ubw = [10*model.p_amb;  20*model.p_amb;   1; 1; 200000*pi/30];
            u_lbw = [1; 0; 20];
            u_ubw = [250; 100; 100];
            xtilde_lbw = x_lbw - x_to; 
            xtilde_ubw = x_ubw - x_to;
            utilde_lbw = u_lbw - u_to;
            utilde_ubw = u_ubw - u_to;

            xtilde_dagger = x_from - x_to; % Disturbed state

            % "Lift" initial conditions
            Uold = casadi.MX.sym('Uold', 3);
            args.p = [args.p; Uold];
%             args.lbw = [args.lbw; u_from - u_to];
%             args.ubw = [args.ubw; u_from - u_to];
%             args.w0 = [args.w0; u_from - u_to];
            
            X0 = casadi.MX.sym('X0', 5);
            args.w = [args.w; X0];
            args.lbw = [args.lbw; xtilde_dagger];
            args.ubw = [args.ubw; xtilde_dagger];
            args.w0 = [args.w0; xtilde_dagger];

            M = 1; % Number of integration steps per sample
            F = integration_function(f, "EF", T_s, M);          
            
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
            
            % Last state objective term
            V_f = Xk'*Q1*Xk;
            args.J = args.J + T_s*V_f;
            
            % qpOASES options
            % opts = struct('qpoases', struct("setPrintLevel", "PL_NONE"));
            
            % Create an NLP solver function
            qp = struct('f', args.J, 'x', args.w, 'p', args.p, 'g', args.G);
            solver = casadi.qpsol('solver', 'qpoases', qp);

            obj.casadi_solver = solver;
            obj.x0 = args.w0;
            obj.lbx = args.lbw;
            obj.ubx = args.ubw;
            obj.lbg = args.lbg;
            obj.ubg = args.ubg;
            
            obj.u_old = u_from - u_to;

        end

        function u = stepImpl(obj,x,t)                       
            w0 = obj.x0;
            lbw = obj.lbx;
            ubw = obj.ubx;
            solver = obj.casadi_solver;
            lbw(1:5) = x - obj.x_ref;
            ubw(1:5) = x - obj.x_ref;
            
            p = [obj.u_old];
            
            if t == 0
                lbw(6:8) = [110.976447879888; 15.3657549771663; 70.8311084176616] - [122.033437081754; 12.7268611588264; 74.8549539702291];
                ubw(6:8) = [110.976447879888; 15.3657549771663; 70.8311084176616] - [122.033437081754; 12.7268611588264; 74.8549539702291];
            end
            
            sol = solver('x0', w0, 'p', p, 'lbx', lbw, 'ubx', ubw,...
                        'lbg', obj.lbg, 'ubg', obj.ubg);
            w_opt = full(sol.x);
            u_opt = [w_opt(6); w_opt(7); w_opt(8)];
            
            u = u_opt + obj.u_ref;
             
            % For integral action 
            obj.u_old = u - obj.u_ref;
        end

        function resetImpl(obj)
            % Initialize discrete-state properties.
        end
    end
end

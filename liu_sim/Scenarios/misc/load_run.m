function opt_signals = load_run(run_name, noob)
% FUNCTION: Loading parameter from previous run and plotting it.
%
% SYNTAX: [output1] = funcion_template(input1);
%
% INPUTS:
%   input1 - Name of .mat file previously created with "save_run.m"
%
% OUTPUTS:
%   output1 - Struct with optimal signals
%
% EXAMPLE:
%   opt_signals = load_run('HEV_0_1800_K2d3');
%
% OTHER FILES REQUIRED:
%   .m files:
%       WHTC_N_table
%       WHTC_dN_table
%   	WHTC_M_table
%
%   .mat files:
%       WHTC_data
%
% SUBFUNCTIONS:
%   none
%
% Author: Hampus Andersson and Fredrik Andersson
% Mars 2018; Last revision: 16-April-2018

%------------- BEGIN CODE --------------

%% LOADING .mat FILE
switch nargin
    case 2
        %% Loading data
        orig_state = warning;
        warning('off','all')
        try
            load(run_name);
        catch
            error('No such file exists! Have you spelled correctly? :)');
        end
        warning(orig_state);
        param.WHTC = createWHTC(engine_map);
        
    case 1
        close all
        
        %% Loading data
        orig_state = warning;
        warning('off','all')
        try
            load(run_name);
        catch
            error('No such file exists! Have you spelled correctly? :)');
        end
        warning(orig_state);
        param.WHTC = createWHTC(engine_map);
        
        %% Plotting
        ld2Plots(opt_signals, engine_map);
        switch numel(opt_signals.x_vec.val(1,:))
            case 9 % ld2_NOx
            case 11 % HEV
                emPlots(v0, opt_signals);
                hevPlots(opt_signals);
            case 21 % EATS_ld2_NOx
                try
                    noxPlots(opt_signals, engine_map);
                catch
                    warning('Missing info for NOx-plots! :(');
                end
                try
                    EATS_plots(opt_signals);
                catch
                    warning('Missing info for EATS-plots! :(');
                end
            case 23 % EATS_HEV
                try
                    noxPlots(opt_signals, engine_map);
                catch
                    warning('Missing info for NOx-plots! :(');
                end
                emPlots(v0, opt_signals);
                hevPlots(opt_signals);
                try
                    EATS_plots(opt_signals);
                catch
                    warning('Missing info for EATS_plots! :(');
                end
            otherwise
                warning('Unknown case!');
        end
        whtcPlots(v0, opt_signals, param);
        
        try
            fprintf(['\n| WHTC from', ' ', num2str(opt_signals.info.t0), ' to', ...
                ' ', num2str(opt_signals.info.tf), ' seconds\n']);
            fprintf(['| K =', ' ', num2str(opt_signals.info.K), ',  d =', ' ', ...
                num2str(opt_signals.info.d), ',  N =', ' ', ...
                num2str(opt_signals.info.N), '\n']);
            fprintf('|\n');
            fprintf(['| Objective L       -', '  ', opt_signals.info.L, '\n']);
            fprintf(['| Objective E       -', '  ', opt_signals.info.E, '\n']);
            fprintf(['| Objective norm    -', '  ', num2str(opt_signals.info.obj_norm), '\n']);
            fprintf('|\n');
            fprintf(['| Const. viol. tol  -', '  ', num2str(opt_signals.info.constr_viol_tol), '\n']);
            fprintf(['| Absolute tol      -', '  ', num2str(opt_signals.info.tol), '\n']);
            fprintf(['| Acceptable tol    -', '  ', num2str(opt_signals.info.acceptable_tol), '\n']);
            fprintf(['| Acceptable iter   -', '  ', num2str(opt_signals.info.acceptable_iter), '\n']);
            fprintf(['| Maximum iter      -', '  ', num2str(opt_signals.info.max_iter), '\n']);
            fprintf('|\n');
            fprintf(['| Duration          -', '  ', num2str(opt_signals.info.opt_duration,5), ' seconds\n']);
            fprintf(['| Time stamp        -', '  ', datestr(opt_signals.info.time_stamp), '\n']);
            
        catch
            warning('Missing field in opt_signals.info...');
        end

        hold off;
end
%------------- END OF CODE --------------
end
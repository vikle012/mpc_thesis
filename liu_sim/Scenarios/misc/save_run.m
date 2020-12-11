% NAME: save_run
%
% PURPOSE:  Saving all required parameters in workspace in a .mat file for
%           plotting later on.
%
% OTHER FILES REQUIRED:
%   .m files:
%       none
%
%   .mat files:
%       none
%
% SUBFUNCTIONS:
%   none
%
% Author: Hampus Andersson and Fredrik Andersson
% March 2018; Last revision: 16-April-2018

%------------- BEGIN CODE --------------

%% DEFINE NAME
switch numel(opt_signals.x_vec.val(1,:))
    case 11 % HEV
        name_of_file = strcat('HEV_', num2str(t0), '_', num2str(round(opt_signals.t_vec.val(end))), ...
            '_K', num2str(K/(tf-t0)), 'd', num2str(d));
    case 9  % ld2_NOx
        name_of_file = strcat('ld2_NOx_', num2str(t0), '_', num2str(round(opt_signals.t_vec.val(end))), ...
            '_K', num2str(K/(tf-t0)), 'd', num2str(d));
    case 21 % EATS_ld2_NOx
        name_of_file = strcat('EATS_ld2_NOx_', num2str(t0), '_', num2str(round(opt_signals.t_vec.val(end))), ...
            '_K', num2str(K/(tf-t0)), 'd', num2str(d));
    case 23 % EATS_HEV
        name_of_file = strcat('EATS_HEV_', num2str(t0), '_', num2str(round(opt_signals.t_vec.val(end))), ...
            '_K', num2str(K/(tf-t0)), 'd', num2str(d));
    otherwise
        fprintf('Neither HEV or ld2_NOx!\n');
        name_of_file = strcat('OTHER', num2str(t0), '_', num2str(round(opt_signals.t_vec.val(end))), ...
            '_K', num2str(K/(tf-t0)), 'd', num2str(d));
end

%% SAVE
orig_state = warning;
warning('off','all')
uisave({'opt_v0', 'opt_signals', 'param', 'v0', 't0', 'tf', 'ocp_sol', 'engine_map'}, name_of_file)
warning(orig_state);

%------------- END OF CODE --------------
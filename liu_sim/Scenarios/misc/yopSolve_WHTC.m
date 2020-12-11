function [ocp_sol, v0out] = yopSolve_WHTC(ocp, v0, K, d, N, nlpOpts)


colOpts = {'mode', 'normal'};
points = 'radau';           % 'legendre' or 'radau'
fprintf('Direct Collocation, creating NLP...\n\n');
[nlp, vars] = directCollocation(ocp, v0, K, N, d, points, colOpts);

%% Modify lower and upper bound to match WHTC

% index_N = 5;      % Index of Engine Speed [1 .. 6 .. 11] among states
% phase = 1;      % There is only one
% tf = v0.T(end);
% dt = tf ./ K;

% for h = 0:K
%     t = h.*dt;
%     N_des = interp1([0; time], [500; speed], t);
%     %% TEST
%     %N_des = ones(1,48).*1200;
%     %N_des(10:30) = 1000;
%     %%
%     N_des = N_des ./ param.ld2.ocp.state_norm(index_N);
%     
%     N_pos = vars.Xi{phase}(h+1, index_N);  % Position in vector
%     nlp.lbw(N_pos) = N_des;
%     nlp.ubw(N_pos) = N_des;
% end

%% Disabling electric traction

% index_M_em = 11;
% 
% for h = 0:K-1    
%     M_em_pos = vars.Xi{phase}(h+1, index_M_em);  % Position in vector
%     nlp.lbw(M_em_pos) = 0;
%     nlp.ubw(M_em_pos) = 0;
% end


%%
fprintf('Solving NLP...\n\n');

nlp_sol = solveNlp(nlp, nlpOpts);
fprintf('\nRetrieving solution...\n');

ocp_sol = retrieveSolution(nlp_sol, ocp, vars, K);
v0out = initFromOpt(ocp_sol);
end


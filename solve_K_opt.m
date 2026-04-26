function [K_P_opt,K_I_opt] = solve_K_opt(J_F0,J_G0,...
                                         rng_seed,optimization_type,alg_type)
    if nargin == 2
        rng_seed = 0; % 随机数种子
        optimization_type = 'fminsearch'; % 采用的优化器算法
        alg_type = 1; % 算法类型
    end
    rng(rng_seed);
    [num_e,num_u] = size(J_G0);
    K_P = randn(num_u,num_e);
    K_I = randn(num_u,num_e);
    K0 = [K_P,K_I];
    K0 = K0(:);
    %% 优化类型
    if strcmp(optimization_type,'fminsearch')
        num_MaxIter = 3e4;
        options = optimset('PlotFcns',@optimplotfval,...
                           'MaxIter',num_MaxIter,...
                           'MaxFunEvals',num_MaxIter);
        [K_opt,lamda_min_opt] = fminsearch(@(K)EVP_obj(K,J_F0,J_G0,alg_type),K0,options);
    elseif strcmp(optimization_type,'ga')
        options = optimoptions('ga','PlotFcn',@gaplotbestf,...
                               'MaxGenerations',2000,'FunctionTolerance',1e-8);
        [K_opt,lamda_min_opt] = ga(@(K)EVP_obj(K,J_F0,J_G0,alg_type),length(K0),options);
    end
    K_opt = reshape(K_opt,[num_u,2*num_e]);
    K_P_opt = K_opt(:,1:num_e);
    K_I_opt = K_opt(:,num_e+1:end);
end
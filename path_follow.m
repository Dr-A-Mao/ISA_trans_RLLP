function out = path_follow(in,P)
    if strcmp(P.method,'VF-Dubins') ||...
            strcmp(P.method,'VF-Line')
        out = path_follow_VF(in,P);
    elseif strcmp(P.method, 'RLLP-x') ||...
           strcmp(P.method, 'RLLP-sin(x)') ||...
           strcmp(P.method, 'RLLP-tan(x)') ||...
           strcmp(P.method, 'RLLP-exp(x)') 
        out = path_follow_RLLP(in,P);
    elseif strcmp(P.method, 'Optimal-Robustness') ||...
           strcmp(P.method, 'Optimal-Energy')||...
           strcmp(P.method, 'Optimal-Attractor')||...
           strcmp(P.method, 'Optimal-Analytical')
        out = path_follow_RLLP_optimal(in,P);
    elseif strcmp(P.method, 'AST_law')
        out = path_follow_AST(in,P);
    elseif strcmp(P.method, 'ISA_law')
        out = path_follow_ISA(in,P);
    elseif strcmp(P.method, 'CJA_law')
        out = path_follow_CJA(in,P);
    elseif strcmp(P.method, 'L1_law')
        out = path_follow_L1(in,P);
    end
    % disp(['path_follow.m成功执行！']);
end
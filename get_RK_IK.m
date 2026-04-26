function [RK,IK] = get_RK_IK(J_F0,J_G0,K_P_opt,K_I_opt)
    K_P_opt_new = K_P_opt;
    K_I_opt_new = K_I_opt;
    [num_e,num_u] = size(J_G0);
    %% 测试A_K(0)的特征值
    A_K0 = [J_F0 + J_G0 * K_P_opt_new, J_G0 * K_I_opt_new;eye(num_e),zeros(num_e)];
    QK_star = get_QK_star(A_K0);
    RK = get_RK(QK_star);
    IK = get_IK(QK_star,RK,A_K0);
    % 打印其最大特征值的实数部分
    disp(['The eigvalues of A_K(0):']);
    disp(eig(A_K0));
    disp(['R_K:',num2str(RK),',I_K:',num2str(IK)]);
end
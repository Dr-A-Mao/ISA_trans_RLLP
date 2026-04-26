%% 代价函数
function max_real_lambda =  EVP_obj(K,J_F0,J_G0,type)
    if nargin == 3
        type = 1;
    end
    % K:(K_P,K_I)的列向量形式
    [num_e,num_u] = size(J_G0);
    K_all = reshape(K,[num_u,2*num_e]);
    K_P = K_all(:,1:num_e);
    K_I = K_all(:,num_e+1:end);
    if type == 1
        EVP_function = [J_F0 + J_F0' + J_G0 * K_P + K_P' * J_G0',...
                     eye(num_e) + J_G0 * K_I;...
                     eye(num_e) + K_I' * J_G0',...
                     zeros(num_e)];
    elseif type == 2
        EVP_function = [J_F0 + J_G0 * K_P,...
                        J_G0 * K_I;...
                        eye(num_e),...
                        zeros(num_e)];
    end
    max_real_lambda = max(real(eig(EVP_function)));
    % !!!优化的是RK(false)还是IK(true)
    IK_type = false;
    if IK_type
        % IK的值为输出
        if max_real_lambda >= 0
            IK = 1e10;
        else
            A_K0 = EVP_function;
            QK_star = get_QK_star(A_K0);
            RK = get_RK(QK_star);
            IK = get_IK(QK_star,RK,A_K0);
            % 下面是修正版本
            % IK = IK * norm(A_K0);
        end
        max_real_lambda = IK;
    end
end
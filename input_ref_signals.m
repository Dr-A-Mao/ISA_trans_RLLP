function ref_data = input_ref_signals(t,Va_c,gamma_c,phi_c)
    if nargin < 1
        t = 20;
        Va_c = 13;
        gamma_c = pi/10;
        phi_c = 0;
    end
    ref_data = [t,Va_c,gamma_c,phi_c];
    % 导出参考数据到workspace中
    % assignin('base', 'ref_data', ref_data);
end
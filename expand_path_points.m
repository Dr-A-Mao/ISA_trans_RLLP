%% 下面函数用来扩展路径点
function path_data = expand_path_points(path_lng,scale)
    % 输入path_lng是列向量
    if nargin == 1
        scale = 10;
    end
    if scale == 1
        path_data = path_lng;
        return;
    end
    t = linspace(0,1,scale)';
    path_data = zeros(scale,length(path_lng)-1);
    for k = 1:length(path_lng)-1
        P1 = path_lng(k);
        P2 = path_lng(k+1);
        % 线性插值化
        P = P1 *(1 - t) + t * P2;
        % 赋值
        path_data(:,k) = P;
    end
    path_data = path_data(:);
end

function in = isPointInConvexPolygon(point, polygon)
    % 输入：
    %   point：待判断的点，格式为 [x, y]
    %   polygon：多边形顶点，格式为 N×2 矩阵（每行一个顶点，按顺序排列）
    % 输出：
    %   in：逻辑值，true 表示在内部或边上，false 表示在外部
    
    n = size(polygon, 1);
    if n < 3
        error('多边形至少需要3个顶点');
    end
    in = true;  % 默认为在内部
    
    for i = 1:n
        % 取第i条边的两个顶点（首尾相连）
        p1 = polygon(i, :);
        p2 = polygon(mod(i, n) + 1, :);  % 最后一条边连接第n个和第1个顶点
        
        % 计算向量叉积：(p2 - p1) × (point - p1)
        % 修正：将 (p2(2) - p1(1)) 改为 (p2(2) - p1(2))
        crossProd = (p2(1) - p1(1)) * (point(2) - p1(2)) ...
                  - (p2(2) - p1(2)) * (point(1) - p1(1));
        
        % 若叉积符号变化（从正变负或负变正），则点在外部
        if i == 1
            sign0 = sign(crossProd);  % 记录第一条边的叉积符号
        else
            % 当前叉积符号与第一条不同，且不同时为0（排除均在边上的情况）
            if sign(crossProd) ~= sign0 && ~(sign(crossProd) == 0 && sign0 == 0)
                in = false;
                break;
            end
        end
    end
end
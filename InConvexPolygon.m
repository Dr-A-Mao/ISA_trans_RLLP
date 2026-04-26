function in = InConvexPolygon(point, obstacle)
    % 输入：
    %   point：待判断的点，格式为 [x, y, z]
    %   polygon：多边形顶点，格式为 N×2 矩阵（每行一个顶点，按顺序排列）
    %   height: 障碍物的高度
    % 输出：
    %   in：逻辑值，true 表示在内部或边上，false 表示在外部
    polygon = obstacle.polygon;
    height = obstacle.height;
    in = isPointInConvexPolygon([point(1),point(2)], polygon);
    if in && (point(3)<=height)
        in = true;
    else
        in = false;
    end
end